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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.units.*;

/**
 * Estimates body kinematics (specific force applied to a body and its angular rate) with respect and resolved
 * along north, east, and down.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Kinematics_NED.m
 */
public class NEDKinematicsEstimator {

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
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double latitude, final double height,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double latitude, final double height,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final NEDVelocity velocity,
                         final NEDVelocity oldVelocity,
                         final double latitude, final double height,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final NEDVelocity velocity,
                         final NEDVelocity oldVelocity,
                         final double latitude, final double height,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final NEDVelocity velocity,
                         final NEDVelocity oldVelocity,
                         final Angle latitude, final Distance height,
                         final Angle oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final NEDVelocity velocity,
                         final NEDVelocity oldVelocity,
                         final Angle latitude, final Distance height,
                         final Angle oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final NEDPosition position, final NEDPosition oldPosition,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final NEDPosition position, final NEDPosition oldPosition,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vn, final Speed ve, final Speed vd,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final NEDPosition position, final NEDPosition oldPosition,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vn, final Speed ve, final Speed vd,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final NEDPosition position, final NEDPosition oldPosition,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final NEDVelocity velocity,
                         final NEDVelocity oldVelocity,
                         final NEDPosition position,
                         final NEDPosition oldPosition,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final NEDVelocity velocity,
                         final NEDVelocity oldVelocity,
                         final NEDPosition position,
                         final NEDPosition oldPosition,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final NEDVelocity oldVelocity,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC,
                oldVelocity, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final NEDVelocity oldVelocity,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final NEDVelocity oldVelocity,
                         final Angle oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final NEDVelocity oldVelocity,
                         final Angle oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVn, final double oldVe, final double oldVd,
                         final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVn, final double oldVe, final double oldVd,
                         final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final NEDVelocity oldVelocity,
                         final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldPosition,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame,
                         final CoordinateTransformation oldC,
                         final NEDVelocity oldVelocity,
                         final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldPosition,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final double vn, final double ve, final double vd,
                         final double latitude, final double height,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, latitude, height, oldFrame,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final double vn, final double ve, final double vd,
                         final double latitude, final double height,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, latitude, height, oldFrame,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final NEDVelocity velocity,
                         final double latitude, final double height,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, latitude, height, oldFrame,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final NEDVelocity velocity,
                         final double latitude, final double height,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, latitude, height, oldFrame,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final NEDVelocity velocity,
                         final Angle latitude, final Distance height,
                         final NEDFrame oldFrame,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, latitude, height, oldFrame,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final NEDVelocity velocity,
                         final Angle latitude, final Distance height,
                         final NEDFrame oldFrame,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, latitude, height, oldFrame,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final double vn, final double ve, final double vd,
                         final NEDPosition position,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final double vn, final double ve, final double vd,
                         final NEDPosition position,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition position,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition position,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final NEDVelocity velocity,
                         final NEDPosition position,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final NEDVelocity velocity,
                         final NEDPosition position,
                         final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval,
                         final NEDFrame frame, final NEDFrame oldFrame,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval,
                         final NEDFrame frame, final NEDFrame oldFrame,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vn, final Speed ve, final Speed vd,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final double latitude, final double height,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vn, final Speed ve, final Speed vd,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final double latitude, final double height,
                         final double oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final Angle latitude, final double height,
                         final Angle oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final Angle latitude, final double height,
                         final Angle oldLatitude, final double oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double latitude, final Distance height,
                         final double oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vn, final double ve, final double vd,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double latitude, final Distance height,
                         final double oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vn, final Speed ve, final Speed vd,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Angle latitude, final Distance height,
                         final Angle oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vn, final Speed ve, final Speed vd,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Angle latitude, final Distance height,
                         final Angle oldLatitude, final Distance oldHeight,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final double latitude, final double height,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final double latitude, final double height,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity velocity,
                                               final NEDVelocity oldVelocity,
                                               final double latitude,
                                               final double height,
                                               final double oldLatitude,
                                               final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity velocity,
                                               final NEDVelocity oldVelocity,
                                               final double latitude,
                                               final double height,
                                               final double oldLatitude,
                                               final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, velocity,
                oldVelocity, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity velocity,
                                               final NEDVelocity oldVelocity,
                                               final Angle latitude,
                                               final Distance height,
                                               final Angle oldLatitude,
                                               final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity velocity,
                                               final NEDVelocity oldVelocity,
                                               final Angle latitude,
                                               final Distance height,
                                               final Angle oldLatitude,
                                               final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final NEDPosition position, final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final NEDPosition position, final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final NEDPosition position, final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final NEDPosition position, final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity velocity,
                                               final NEDVelocity oldVelocity,
                                               final NEDPosition position,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, position, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity velocity,
                                               final NEDVelocity oldVelocity,
                                               final NEDPosition position,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                velocity, oldVelocity, position, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity oldVelocity,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVelocity, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity oldVelocity,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity oldVelocity,
                                               final Angle oldLatitude, final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity oldVelocity,
                                               final Angle oldLatitude, final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVn, oldVe, oldVd, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVn, oldVe, oldVd, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVn, oldVe, oldVd, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVn, oldVe, oldVd, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity oldVelocity,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVelocity, oldPosition);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final CoordinateTransformation oldC,
                                               final NEDVelocity oldVelocity,
                                               final NEDPosition oldPosition) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVelocity, oldPosition);
    }


    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final double vn, final double ve, final double vd,
                                               final double latitude, final double height,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final double vn, final double ve, final double vd,
                                               final double latitude, final double height,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final NEDVelocity velocity,
                                               final double latitude, final double height,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                latitude, height, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final NEDVelocity velocity,
                                               final double latitude, final double height,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                latitude, height, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final NEDVelocity velocity,
                                               final Angle latitude, final Distance height,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                latitude, height, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final NEDVelocity velocity,
                                               final Angle latitude, final Distance height,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                latitude, height, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final double vn, final double ve, final double vd,
                                               final NEDPosition position,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd,
                position, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final double vn, final double ve, final double vd,
                                               final NEDPosition position,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd,
                position, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final NEDPosition position,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd,
                position, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final NEDPosition position,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd,
                position, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final NEDVelocity velocity,
                                               final NEDPosition position,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                position, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final NEDVelocity velocity,
                                               final NEDPosition position,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                position, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final NEDFrame frame,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final NEDFrame frame,
                                               final NEDFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final double latitude, final double height,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final double latitude, final double height,
                                               final double oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final Angle latitude, final double height,
                                               final Angle oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final Angle latitude, final double height,
                                               final Angle oldLatitude, final double oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final double latitude, final Distance height,
                                               final double oldLatitude, final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vn, final double ve, final double vd,
                                               final double oldVn, final double oldVe, final double oldVd,
                                               final double latitude, final Distance height,
                                               final double oldLatitude, final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final Angle latitude, final Distance height,
                                               final Angle oldLatitude, final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vn, final Speed ve, final Speed vd,
                                               final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                               final Angle latitude, final Distance height,
                                               final Angle oldLatitude, final Distance oldHeight) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final double latitude, final double height,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {

        if (timeInterval < 0.0
                || !NEDFrame.isValidCoordinateTransformation(c)
                || !NEDFrame.isValidCoordinateTransformation(oldC)) {
            throw new IllegalArgumentException();
        }

        if (timeInterval > 0.0) {
            try {
                // From (2.123), determine the angular rate of the NED frame
                // with respect the ECI frame, resolved about NED
                final Matrix omegaIen = new Matrix(ROWS, 1);
                omegaIen.setElementAtIndex(0,
                        EARTH_ROTATION_RATE * Math.cos(oldLatitude));
                omegaIen.setElementAtIndex(2,
                        -EARTH_ROTATION_RATE * Math.sin(oldLatitude));

                // From (5.44), determine the angular rate of the NED frame
                // with respect the NED frame, resolved about NED
                final RadiiOfCurvature oldRadiiOfCurvature = RadiiOfCurvatureEstimator
                        .estimateRadiiOfCurvatureAndReturnNew(oldLatitude);
                final double oldRn = oldRadiiOfCurvature.getRn();
                final double oldRe = oldRadiiOfCurvature.getRe();

                final RadiiOfCurvature radiiOfCurvature = RadiiOfCurvatureEstimator
                        .estimateRadiiOfCurvatureAndReturnNew(latitude);
                final double rn = radiiOfCurvature.getRn();
                final double re = radiiOfCurvature.getRe();

                final Matrix oldOmegaEnN = new Matrix(ROWS, 1);
                oldOmegaEnN.setElementAtIndex(0,
                        oldVe / (oldRe + oldHeight));
                oldOmegaEnN.setElementAtIndex(1,
                        -oldVn / (oldRn + oldHeight));
                oldOmegaEnN.setElementAtIndex(2,
                        -oldVe * Math.tan(oldLatitude) /
                                (oldRe + oldHeight));

                final Matrix omegaEnN = new Matrix(ROWS, 1);
                omegaEnN.setElementAtIndex(0, ve / (re + height));
                omegaEnN.setElementAtIndex(1, -vn / (rn + height));
                omegaEnN.setElementAtIndex(2,
                        -ve * Math.tan(latitude) / (re + height));

                // Obtain coordinate transformation matrix from the old attitude (with
                // respect the inertial frame) to the new using (5.77)
                final Matrix cbn = c.getMatrix();
                cbn.transpose();

                final Matrix omega = omegaIen.addAndReturnNew(
                        omegaEnN.multiplyByScalarAndReturnNew(0.5));
                omega.add(oldOmegaEnN.multiplyByScalarAndReturnNew(0.5));
                final Matrix skew1 = Utils.skewMatrix(omega);
                skew1.multiplyByScalar(timeInterval);

                final Matrix tmp1 = Matrix.identity(ROWS, ROWS);
                tmp1.subtract(skew1);

                final Matrix oldCbn = oldC.getMatrix();
                cbn.multiply(tmp1);
                cbn.multiply(oldCbn);
                // cbn contains transformation matrix from the old to the new attitude (cOldNew)

                // Calculate the approximate angular rate with respect an inertial frame
                final Matrix alphaIbb = new Matrix(ROWS, 1);
                alphaIbb.setElementAtIndex(0,
                        0.5 * (cbn.getElementAt(1, 2) - cbn.getElementAt(2, 1)));
                alphaIbb.setElementAtIndex(1,
                        0.5 * (cbn.getElementAt(2, 0) - cbn.getElementAt(0, 2)));
                alphaIbb.setElementAtIndex(2,
                        0.5 * (cbn.getElementAt(0, 1) - cbn.getElementAt(1, 0)));

                // Calculate and apply the scaling factor
                final double temp = Math.acos(0.5 * (Utils.trace(cbn) - 1.0));
                if (temp > SCALING_THRESHOLD) {
                    // Scaling is 1 if temp is less than this
                    alphaIbb.multiplyByScalar(temp / Math.sin(temp));
                }

                // Calculate the angular rate
                final double angularRateX = alphaIbb.getElementAtIndex(0) / timeInterval;
                final double angularRateY = alphaIbb.getElementAtIndex(1) / timeInterval;
                final double angularRateZ = alphaIbb.getElementAtIndex(2) / timeInterval;

                // Calculate the specific force resolved about NED-frame axes
                // From (5.54)
                final Matrix tmp2 = new Matrix(ROWS, 1);
                tmp2.setElementAtIndex(0, (vn - oldVn) / timeInterval);
                tmp2.setElementAtIndex(1, (ve - oldVe) / timeInterval);
                tmp2.setElementAtIndex(2, (vd - oldVd) / timeInterval);

                final NEDGravity gravity = NEDGravityEstimator.estimateGravityAndReturnNew(
                        oldLatitude, oldHeight);
                final Matrix g = gravity.asMatrix();

                final Matrix oldVebn = new Matrix(ROWS, 1);
                oldVebn.setElementAtIndex(0, oldVn);
                oldVebn.setElementAtIndex(1, oldVe);
                oldVebn.setElementAtIndex(2, oldVd);

                final Matrix skew2 = Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(
                        omegaIen.multiplyByScalarAndReturnNew(2.0)));
                skew2.multiply(oldVebn);

                tmp2.subtract(g);
                tmp2.add(skew2);
                // tmp2 now contains specific force resolved about NED (fIbn)

                // Calculate the average body-to-NED coordinate transformation
                // matrix over the update interval using (5.84) and (5.86)
                final double magAlpha = Utils.normF(alphaIbb);
                final Matrix skewAlpha = Utils.skewMatrix(alphaIbb);

                oldOmegaEnN.add(omegaIen);
                final Matrix tmp3 = Utils.skewMatrix(oldOmegaEnN);
                tmp3.multiplyByScalar(0.5);
                tmp3.multiply(oldCbn);

                if (magAlpha > ALPHA_THRESHOLD) {
                    final double magAlpha2 = magAlpha * magAlpha;
                    final double value1 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                    final double value2 = (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2;

                    final Matrix tmp4 = skewAlpha.multiplyByScalarAndReturnNew(value1);
                    final Matrix tmp5 = skewAlpha.multiplyByScalarAndReturnNew(value2);
                    tmp5.multiply(skewAlpha);

                    final Matrix tmp6 = Matrix.identity(ROWS, ROWS);
                    tmp6.add(tmp4);
                    tmp6.add(tmp5);

                    oldCbn.multiply(tmp6);
                }

                oldCbn.subtract(tmp3);
                // oldCbn now contains average body-to-NED (aveCbn)

                // Transform specific force to body-frame resolving axes using (5.81)
                final Matrix fIbb = Utils.inverse(oldCbn);
                fIbb.multiply(tmp2);

                final double specificForceX = fIbb.getElementAtIndex(0);
                final double specificForceY = fIbb.getElementAtIndex(1);
                final double specificForceZ = fIbb.getElementAtIndex(2);

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
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final double latitude, final double height,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity velocity,
                                          final NEDVelocity oldVelocity,
                                          final double latitude, final double height,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity.getVn(), velocity.getVe(), velocity.getVd(),
                oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity velocity,
                                          final NEDVelocity oldVelocity,
                                          final double latitude, final double height,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, velocity, oldVelocity, latitude,
                height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity velocity,
                                          final NEDVelocity oldVelocity,
                                          final Angle latitude, final Distance height,
                                          final Angle oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                convertAngle(latitude), convertDistance(height),
                convertAngle(oldLatitude), convertDistance(oldHeight),
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity velocity,
                                          final NEDVelocity oldVelocity,
                                          final Angle latitude, final Distance height,
                                          final Angle oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final NEDPosition position, final NEDPosition oldPosition,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position.getLatitude(), position.getHeight(),
                oldPosition.getLatitude(), oldPosition.getHeight(),
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final NEDPosition position, final NEDPosition oldPosition,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final NEDPosition position, final NEDPosition oldPosition,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, convertSpeed(vn),
                convertSpeed(ve), convertSpeed(vd), convertSpeed(oldVn),
                convertSpeed(oldVe), convertSpeed(oldVd), position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final NEDPosition position, final NEDPosition oldPosition,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity velocity,
                                          final NEDVelocity oldVelocity,
                                          final NEDPosition position,
                                          final NEDPosition oldPosition,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                position.getLatitude(), position.getHeight(),
                oldPosition.getLatitude(), oldPosition.getHeight(),
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity velocity,
                                          final NEDVelocity oldVelocity,
                                          final NEDPosition position,
                                          final NEDPosition oldPosition,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, velocity, oldVelocity,
                position, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame.getCoordinateTransformation(), oldC,
                frame.getVn(), frame.getVe(), frame.getVd(), oldVn, oldVe, oldVd,
                frame.getLatitude(), frame.getHeight(), oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity oldVelocity,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity oldVelocity,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldC, oldVelocity, oldLatitude, oldHeight,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity oldVelocity,
                                          final Angle oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                convertAngle(oldLatitude), convertDistance(oldHeight), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity oldVelocity,
                                          final Angle oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition.getLatitude(), oldPosition.getHeight(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, convertSpeed(oldVn),
                convertSpeed(oldVe), convertSpeed(oldVd), oldPosition,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldC,
                oldVn, oldVe, oldVd, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity oldVelocity,
                                          final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame,
                                          final CoordinateTransformation oldC,
                                          final NEDVelocity oldVelocity,
                                          final NEDPosition oldPosition, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldC, oldVelocity, oldPosition, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final double vn, final double ve, final double vd,
                                          final double latitude, final double height,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldFrame.getCoordinateTransformation(),
                vn, ve, vd, oldFrame.getVn(), oldFrame.getVe(), oldFrame.getVd(),
                latitude, height, oldFrame.getLatitude(), oldFrame.getHeight(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final double vn, final double ve, final double vd,
                                          final double latitude, final double height,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, vn, ve, vd, latitude, height,
                oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final NEDVelocity velocity,
                                          final double latitude, final double height,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity.getVn(), velocity.getVe(), velocity.getVd(),
                latitude, height, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final NEDVelocity velocity,
                                          final double latitude, final double height,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, velocity, latitude, height, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final NEDVelocity velocity,
                                          final Angle latitude, final Distance height,
                                          final NEDFrame oldFrame,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, convertAngle(latitude),
                convertDistance(height), oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final NEDVelocity velocity,
                                          final Angle latitude, final Distance height,
                                          final NEDFrame oldFrame,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, velocity, latitude,
                height, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final double vn, final double ve, final double vd,
                                          final NEDPosition position,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, vn, ve, vd, position.getLatitude(), position.getHeight(),
                oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final double vn, final double ve, final double vd,
                                          final NEDPosition position,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, vn, ve, vd, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final NEDPosition position,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, convertSpeed(vn), convertSpeed(ve),
                convertSpeed(vd), position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final NEDPosition position,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, vn, ve, vd,
                position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final NEDVelocity velocity,
                                          final NEDPosition position,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, velocity, position.getLatitude(), position.getHeight(),
                oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final NEDVelocity velocity,
                                          final NEDPosition position,
                                          final NEDFrame oldFrame, final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, velocity, position, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final NEDFrame frame, final NEDFrame oldFrame,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame.getCoordinateTransformation(),
                frame.getVn(), frame.getVe(), frame.getVd(),
                frame.getLatitude(), frame.getHeight(), oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final NEDFrame frame, final NEDFrame oldFrame,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final double latitude, final double height,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, convertSpeed(vn), convertSpeed(ve),
                convertSpeed(vd), convertSpeed(oldVn), convertSpeed(oldVe),
                convertSpeed(oldVd), latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final double latitude, final double height,
                                          final double oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final Angle latitude, final double height,
                                          final Angle oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                convertAngle(latitude), height, convertAngle(oldLatitude), oldHeight,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final Angle latitude, final double height,
                                          final Angle oldLatitude, final double oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final double latitude, final Distance height,
                                          final double oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, convertDistance(height), oldLatitude,
                convertDistance(oldHeight), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vn, final double ve, final double vd,
                                          final double oldVn, final double oldVe, final double oldVd,
                                          final double latitude, final Distance height,
                                          final double oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd, oldVn,
                oldVe, oldVd, latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final Angle latitude, final Distance height,
                                          final Angle oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, convertSpeed(vn), convertSpeed(ve),
                convertSpeed(vd), convertSpeed(oldVn), convertSpeed(oldVe),
                convertSpeed(oldVd), convertAngle(latitude), convertDistance(height),
                convertAngle(oldLatitude), convertDistance(oldHeight), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vn, final Speed ve, final Speed vd,
                                          final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                          final Angle latitude, final Distance height,
                                          final Angle oldLatitude, final Distance oldHeight,
                                          final BodyKinematics result) {
        estimateKinematics(convertTime(timeInterval), c, oldC, vn, ve, vd, oldVn, oldVe,
                oldVd, latitude, height, oldLatitude, oldHeight, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final double latitude, final double height,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final double latitude, final double height,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity velocity,
                                                                final NEDVelocity oldVelocity,
                                                                final double latitude,
                                                                final double height,
                                                                final double oldLatitude,
                                                                final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity velocity,
                                                                final NEDVelocity oldVelocity,
                                                                final double latitude,
                                                                final double height,
                                                                final double oldLatitude,
                                                                final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity velocity,
                                                                final NEDVelocity oldVelocity,
                                                                final Angle latitude,
                                                                final Distance height,
                                                                final Angle oldLatitude,
                                                                final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, latitude,
                height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity velocity,
                                                                final NEDVelocity oldVelocity,
                                                                final Angle latitude,
                                                                final Distance height,
                                                                final Angle oldLatitude,
                                                                final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, latitude,
                height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final NEDPosition position, final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final NEDPosition position, final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final NEDPosition position, final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final NEDPosition position, final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity velocity,
                                                                final NEDVelocity oldVelocity,
                                                                final NEDPosition position,
                                                                final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                position, oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity velocity,
                                                                final NEDVelocity oldVelocity,
                                                                final NEDPosition position,
                                                                final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity,
                position, oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity oldVelocity,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity oldVelocity,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity oldVelocity,
                                                                final Angle oldLatitude, final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final NEDVelocity oldVelocity,
                                                                final Angle oldLatitude, final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final NEDFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                    final NEDFrame frame,
                                                    final CoordinateTransformation oldC,
                                                    final NEDVelocity oldVelocity,
                                                    final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldPosition,
                result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param oldVelocity  previous velocity of body frame with respect NED frame, resolved along north, east and
     *                     down.
     * @param oldPosition  previous curvilinear position expressed in terms of latitude, longitude and height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                    final NEDFrame frame,
                                                    final CoordinateTransformation oldC,
                                                    final NEDVelocity oldVelocity,
                                                    final NEDPosition oldPosition) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldPosition,
                result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final double vn, final double ve, final double vd,
                                                                final double latitude, final double height,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, vn, ve, vd, latitude, height, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final double vn, final double ve, final double vd,
                                                                final double latitude, final double height,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, vn, ve, vd, latitude, height, oldFrame,
                result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final NEDVelocity velocity,
                                                                final double latitude, final double height,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, velocity, latitude, height,
                oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final NEDVelocity velocity,
                                                                final double latitude, final double height,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, velocity, latitude, height,
                oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final NEDVelocity velocity,
                                                                final Angle latitude, final Distance height,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, velocity, latitude, height,
                oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final NEDVelocity velocity,
                                                                final Angle latitude, final Distance height,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, velocity, latitude, height,
                oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final double vn, final double ve, final double vd,
                                                                final NEDPosition position,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame,
                result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final double vn, final double ve, final double vd,
                                                                final NEDPosition position,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame,
                result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final NEDPosition position,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, vn, ve, vd, position,
                oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final NEDPosition position,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, vn, ve, vd, position,
                oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final NEDVelocity velocity,
                                                                final NEDPosition position,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, velocity, position, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param velocity     velocity of body frame with respect NED frame, resolved along north, east and down.
     * @param position     current curvilinear position expressed in terms of latitude, longitude and height.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final NEDVelocity velocity,
                                                                final NEDPosition position,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, velocity, position, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final NEDFrame frame,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param frame        NED frame containing current position, velocity and body-to-NED coordinate transformation.
     * @param oldFrame     NED frame containing previous position, velocity and body-to-NED coordinate transformation.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final NEDFrame frame,
                                                                final NEDFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final double latitude, final double height,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final double latitude, final double height,
                                                                final double oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final Angle latitude, final double height,
                                                                final Angle oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param height       current height expressed in meters (m).
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height expressed in meters (m).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final Angle latitude, final double height,
                                                                final Angle oldLatitude, final double oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final double latitude, final Distance height,
                                                                final double oldLatitude, final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down and expressed in meters per second (m/s).
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down and expressed in meters per second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param height       current height.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vn, final double ve, final double vd,
                                                                final double oldVn, final double oldVe, final double oldVd,
                                                                final double latitude, final Distance height,
                                                                final double oldLatitude, final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final Angle latitude, final Distance height,
                                                                final Angle oldLatitude, final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)
     * with respect NED frame and resolved along body-frame axes, averaged over time interval.
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-NED coordinate transformation.
     * @param oldC         previous body-to-NED coordinate transformation.
     * @param vn           north coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param ve           east coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param vd           down coordinate of velocity of body frame with respect NED frame, resolved along
     *                     north, east, and down.
     * @param oldVn        north coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVe        east coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param oldVd        down coordinate of previous velocity of body frame with respect NED frame, resolved
     *                     along north, east, and down.
     * @param latitude     current latitude.
     * @param height       current height.
     * @param oldLatitude  previous latitude.
     * @param oldHeight    previous height.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinate transformation
     *                                  matrices are not NED frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vn, final Speed ve, final Speed vd,
                                                                final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                                final Angle latitude, final Distance height,
                                                                final Angle oldLatitude, final Distance oldHeight) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight, result);
        return result;
    }

    /**
     * Converts a time instance into seconds.
     *
     * @param time time to be converted.
     * @return time value expressed in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(),
                TimeUnit.SECOND);
    }

    /**
     * Converts a speed instance into meters per second.
     *
     * @param speed speed to be converted.
     * @return speed value expressed in meters per second.
     */
    private static double convertSpeed(final Speed speed) {
        return SpeedConverter.convert(speed.getValue().doubleValue(), speed.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Converts an angle instance into radians.
     *
     * @param angle angle to be converted.
     * @return angle value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(), angle.getUnit(),
                AngleUnit.RADIANS);
    }

    /**
     * Converts a distance instance into meters.
     *
     * @param distance distance to be converted.
     * @return distance value expressed in meters.
     */
    private static double convertDistance(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }
}
