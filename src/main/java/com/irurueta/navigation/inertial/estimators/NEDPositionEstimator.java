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

import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.units.*;

/**
 * Estimates curvilinear position by integrating the velocity.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
 */
@SuppressWarnings("WeakerAccess")
public class NEDPositionEstimator {

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double vn, final double ve, final double vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double vn, final double ve, final double vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double vn, final double ve, final double vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double vn, final double ve, final double vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final NEDPosition oldPosition,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double vn, final double ve, final double vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final NEDPosition oldPosition,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double vn, final double ve, final double vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final NEDPosition oldPosition,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final NEDPosition oldPosition,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Speed vn, final Speed ve, final Speed vd,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval,
                         final double oldLatitude, final double oldLongitude,
                         final double oldHeight, final NEDVelocity oldVelocity,
                         final NEDVelocity velocity, final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final NEDVelocity oldVelocity, final NEDVelocity velocity,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval,
                         final Angle oldLatitude, final Angle oldLongitude,
                         final Distance oldHeight, final NEDVelocity oldVelocity,
                         final NEDVelocity velocity, final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final NEDVelocity oldVelocity, final NEDVelocity velocity,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval, final NEDPosition oldPosition,
                         final NEDVelocity oldVelocity, final NEDVelocity velocity,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval, final NEDPosition oldPosition,
                         final NEDVelocity oldVelocity, final NEDVelocity velocity,
                         final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final double oldLatitude,
                                            final double oldLongitude,
                                            final double oldHeight,
                                            final double oldVn,
                                            final double oldVe,
                                            final double oldVd,
                                            final double vn,
                                            final double ve,
                                            final double vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final double oldLatitude,
                                            final double oldLongitude,
                                            final double oldHeight,
                                            final double oldVn,
                                            final double oldVe,
                                            final double oldVd,
                                            final double vn,
                                            final double ve,
                                            final double vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final Angle oldLatitude,
                                            final Angle oldLongitude,
                                            final Distance oldHeight,
                                            final Speed oldVn,
                                            final Speed oldVe,
                                            final Speed oldVd,
                                            final Speed vn,
                                            final Speed ve,
                                            final Speed vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final Angle oldLatitude,
                                            final Angle oldLongitude,
                                            final Distance oldHeight,
                                            final Speed oldVn,
                                            final Speed oldVe,
                                            final Speed oldVd,
                                            final Speed vn,
                                            final Speed ve,
                                            final Speed vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final Angle oldLatitude,
                                            final Angle oldLongitude,
                                            final Distance oldHeight,
                                            final double oldVn,
                                            final double oldVe,
                                            final double oldVd,
                                            final double vn,
                                            final double ve,
                                            final double vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final Angle oldLatitude,
                                            final Angle oldLongitude,
                                            final Distance oldHeight,
                                            final double oldVn,
                                            final double oldVe,
                                            final double oldVd,
                                            final double vn,
                                            final double ve,
                                            final double vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final double oldLatitude,
                                            final double oldLongitude,
                                            final double oldHeight,
                                            final Speed oldVn,
                                            final Speed oldVe,
                                            final Speed oldVd,
                                            final Speed vn,
                                            final Speed ve,
                                            final Speed vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final double oldLatitude,
                                            final double oldLongitude,
                                            final double oldHeight,
                                            final Speed oldVn,
                                            final Speed oldVe,
                                            final Speed oldVd,
                                            final Speed vn,
                                            final Speed ve,
                                            final Speed vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final NEDPosition oldPosition,
                                            final double oldVn,
                                            final double oldVe,
                                            final double oldVd,
                                            final double vn,
                                            final double ve,
                                            final double vd) {
        return estimatePositionAndReturnNew(timeInterval,
                oldPosition, oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final NEDPosition oldPosition,
                                            final double oldVn,
                                            final double oldVe,
                                            final double oldVd,
                                            final double vn,
                                            final double ve,
                                            final double vd) {
        return estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final NEDPosition oldPosition,
                                            final Speed oldVn,
                                            final Speed oldVe,
                                            final Speed oldVd,
                                            final Speed vn,
                                            final Speed ve,
                                            final Speed vd) {
        return estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final NEDPosition oldPosition,
                                            final Speed oldVn,
                                            final Speed oldVe,
                                            final Speed oldVd,
                                            final Speed vn,
                                            final Speed ve,
                                            final Speed vd) {
        return estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final double oldLatitude,
                                            final double oldLongitude,
                                            final double oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final NEDVelocity velocity) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final double oldLatitude,
                                            final double oldLongitude,
                                            final double oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final NEDVelocity velocity) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final Angle oldLatitude,
                                            final Angle oldLongitude,
                                            final Distance oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final NEDVelocity velocity) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final Angle oldLatitude,
                                            final Angle oldLongitude,
                                            final Distance oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final NEDVelocity velocity) {
        return estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final double timeInterval,
                                            final NEDPosition oldPosition,
                                            final NEDVelocity oldVelocity,
                                            final NEDVelocity velocity) {
        return estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldVelocity, velocity);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public NEDPosition estimateAndReturnNew(final Time timeInterval,
                                            final NEDPosition oldPosition,
                                            final NEDVelocity oldVelocity,
                                            final NEDVelocity velocity) {
        return estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldVelocity, velocity);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double vn, final double ve, final double vd,
                                        final NEDPosition result) {

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        // Calculate meridian and transverse radii of curvature
        final RadiiOfCurvature oldRadii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(oldLatitude);
        final double oldRn = oldRadii.getRn();
        final double oldRe = oldRadii.getRe();

        // Update height using (5.56)
        final double height = oldHeight - 0.5 * timeInterval * (oldVd + vd);

        // Update latitude using (5.56)
        final double latitude = oldLatitude
                + 0.5 * timeInterval * (oldVn / (oldRn + oldHeight)
                + vn / (oldRn + height));

        // Calculate meridian and transverse radii of curvature
        final RadiiOfCurvature radii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(latitude);
        final double re = radii.getRe();

        // Update longitude using (5.56)
        final double longitude = oldLongitude
                + 0.5 * timeInterval * (oldVe / ((oldRe + oldHeight) * Math.cos(oldLatitude))
                + ve / ((re + height) * Math.cos(latitude)));

        result.setCoordinates(latitude, longitude, height);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double vn, final double ve, final double vd,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude,
                oldHeight, oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Speed vn, final Speed ve, final Speed vd,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, convertAngleToDouble(oldLatitude),
                convertAngleToDouble(oldLongitude), convertDistanceToDouble(oldHeight),
                convertSpeedToDouble(oldVn), convertSpeedToDouble(oldVe),
                convertSpeedToDouble(oldVd), convertSpeedToDouble(vn),
                convertSpeedToDouble(ve), convertSpeedToDouble(vd),
                result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Speed vn, final Speed ve, final Speed vd,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude,
                oldHeight, oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double vn, final double ve, final double vd,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, convertAngleToDouble(oldLatitude),
                convertAngleToDouble(oldLongitude), convertDistanceToDouble(oldHeight),
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double vn, final double ve, final double vd,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude,
                oldHeight, oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Speed vn, final Speed ve, final Speed vd,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                convertSpeedToDouble(oldVn), convertSpeedToDouble(oldVe),
                convertSpeedToDouble(oldVd), convertSpeedToDouble(vn),
                convertSpeedToDouble(ve), convertSpeedToDouble(vd), result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Speed vn, final Speed ve, final Speed vd,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude,
                oldHeight, oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval,
                                        final NEDPosition oldPosition,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double vn, final double ve, final double vd,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition.getLatitude(),
                oldPosition.getLongitude(), oldPosition.getHeight(), oldVn, oldVe, oldVd,
                vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval,
                                        final NEDPosition oldPosition,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double vn, final double ve, final double vd,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval,
                                        final NEDPosition oldPosition,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Speed vn, final Speed ve, final Speed vd,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition, convertSpeedToDouble(oldVn),
                convertSpeedToDouble(oldVe), convertSpeedToDouble(oldVd),
                convertSpeedToDouble(vn), convertSpeedToDouble(ve),
                convertSpeedToDouble(vd), result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval,
                                        final NEDPosition oldPosition,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Speed vn, final Speed ve, final Speed vd,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval,
                                        final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final NEDVelocity velocity,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                velocity.getVn(), velocity.getVe(), velocity.getVd(), result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval,
                                        final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final NEDVelocity velocity,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude,
                oldHeight, oldVelocity, velocity, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval,
                                        final Angle oldLatitude, final Angle oldLongitude,
                                        final Distance oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final NEDVelocity velocity,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, convertAngleToDouble(oldLatitude),
                convertAngleToDouble(oldLongitude), convertDistanceToDouble(oldHeight),
                oldVelocity, velocity, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval,
                                        final Angle oldLatitude, final Angle oldLongitude,
                                        final Distance oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final NEDVelocity velocity,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude,
                oldHeight, oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final double timeInterval,
                                        final NEDPosition oldPosition,
                                        final NEDVelocity oldVelocity,
                                        final NEDVelocity velocity,
                                        final NEDPosition result) {
        estimatePosition(timeInterval, oldPosition.getLatitude(),
                oldPosition.getLongitude(), oldPosition.getHeight(),
                oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param result       instance where updated curvilinear position will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimatePosition(final Time timeInterval,
                                        final NEDPosition oldPosition,
                                        final NEDVelocity oldVelocity,
                                        final NEDVelocity velocity,
                                        final NEDPosition result) {
        estimatePosition(convertTimeToDouble(timeInterval), oldPosition,
                oldVelocity, velocity, result);
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final double oldLatitude,
                                                           final double oldLongitude,
                                                           final double oldHeight,
                                                           final double oldVn,
                                                           final double oldVe,
                                                           final double oldVd,
                                                           final double vn,
                                                           final double ve,
                                                           final double vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final double oldLatitude,
                                                           final double oldLongitude,
                                                           final double oldHeight,
                                                           final double oldVn,
                                                           final double oldVe,
                                                           final double oldVd,
                                                           final double vn,
                                                           final double ve,
                                                           final double vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final Angle oldLatitude,
                                                           final Angle oldLongitude,
                                                           final Distance oldHeight,
                                                           final Speed oldVn,
                                                           final Speed oldVe,
                                                           final Speed oldVd,
                                                           final Speed vn,
                                                           final Speed ve,
                                                           final Speed vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final Angle oldLatitude,
                                                           final Angle oldLongitude,
                                                           final Distance oldHeight,
                                                           final Speed oldVn,
                                                           final Speed oldVe,
                                                           final Speed oldVd,
                                                           final Speed vn,
                                                           final Speed ve,
                                                           final Speed vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final Angle oldLatitude,
                                                           final Angle oldLongitude,
                                                           final Distance oldHeight,
                                                           final double oldVn,
                                                           final double oldVe,
                                                           final double oldVd,
                                                           final double vn,
                                                           final double ve,
                                                           final double vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final Angle oldLatitude,
                                                           final Angle oldLongitude,
                                                           final Distance oldHeight,
                                                           final double oldVn,
                                                           final double oldVe,
                                                           final double oldVd,
                                                           final double vn,
                                                           final double ve,
                                                           final double vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final double oldLatitude,
                                                           final double oldLongitude,
                                                           final double oldHeight,
                                                           final Speed oldVn,
                                                           final Speed oldVe,
                                                           final Speed oldVd,
                                                           final Speed vn,
                                                           final Speed ve,
                                                           final Speed vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final double oldLatitude,
                                                           final double oldLongitude,
                                                           final double oldHeight,
                                                           final Speed oldVn,
                                                           final Speed oldVe,
                                                           final Speed oldVd,
                                                           final Speed vn,
                                                           final Speed ve,
                                                           final Speed vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final NEDPosition oldPosition,
                                                           final double oldVn,
                                                           final double oldVe,
                                                           final double oldVd,
                                                           final double vn,
                                                           final double ve,
                                                           final double vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd,
                result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final NEDPosition oldPosition,
                                                           final double oldVn,
                                                           final double oldVe,
                                                           final double oldVd,
                                                           final double vn,
                                                           final double ve,
                                                           final double vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd,
                result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final NEDPosition oldPosition,
                                                           final Speed oldVn,
                                                           final Speed oldVe,
                                                           final Speed oldVd,
                                                           final Speed vn,
                                                           final Speed ve,
                                                           final Speed vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd,
                result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param vn           current velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param ve           current velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param vd           current velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final NEDPosition oldPosition,
                                                           final Speed oldVn,
                                                           final Speed oldVe,
                                                           final Speed oldVd,
                                                           final Speed vn,
                                                           final Speed ve,
                                                           final Speed vd) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final double oldLatitude,
                                                           final double oldLongitude,
                                                           final double oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDVelocity velocity) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final double oldLatitude,
                                                           final double oldLongitude,
                                                           final double oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDVelocity velocity) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final Angle oldLatitude,
                                                           final Angle oldLongitude,
                                                           final Distance oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDVelocity velocity) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude.
     * @param oldLongitude previous longitude.
     * @param oldHeight    previous height.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final Angle oldLatitude,
                                                           final Angle oldLongitude,
                                                           final Distance oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDVelocity velocity) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final double timeInterval,
                                                           final NEDPosition oldPosition,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDVelocity velocity) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldPosition, oldVelocity, velocity, result);
        return result;
    }

    /**
     * Estimates curvilinear position by integrating the velocity.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param velocity     current velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @return new updated curvilinear position.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static NEDPosition estimatePositionAndReturnNew(final Time timeInterval,
                                                           final NEDPosition oldPosition,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDVelocity velocity) {
        final NEDPosition result = new NEDPosition();
        estimatePosition(timeInterval, oldPosition, oldVelocity, velocity,
                result);
        return result;
    }

    /**
     * Converts provided time instance to seconds (s).
     *
     * @param time time instance to be converted.
     * @return provided time value expressed in seconds.
     */
    private static double convertTimeToDouble(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(),
                TimeUnit.SECOND);
    }

    /**
     * Converts provided angle instance to radians (rad).
     *
     * @param angle angle instance to be converted.
     * @return provided angle value expressed in radians.
     */
    private static double convertAngleToDouble(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(), angle.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Converts provided distance instance to meters (m).
     *
     * @param distance distance instance to be converted.
     * @return provided distance value expressed in meters.
     */
    private static double convertDistanceToDouble(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(), distance.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Converts provided speed instance to meters per second (m/s).
     *
     * @param speed speed instance to be converted.
     * @return provided speed value expressed in meters per second.
     */
    private static double convertSpeedToDouble(final Speed speed) {
        return SpeedConverter.convert(speed.getValue().doubleValue(), speed.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }
}
