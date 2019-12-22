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

import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.units.*;

/**
 * Estimates velocity by differentiating latitude, longitude and height over
 * a time interval.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Velocity_from_curvilinear.m
 */
@SuppressWarnings("WeakerAccess")
public class NEDVelocityEstimator {

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double latitude, final double longitude,
                         final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final double latitude, final double longitude,
                         final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final double oldVn, final double oldVe, final double oldVd,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final NEDVelocity oldVelocity,
                         final double latitude, final double longitude,
                         final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final double oldLatitude,
                         final double oldLongitude, final double oldHeight,
                         final NEDVelocity oldVelocity,
                         final double latitude, final double longitude,
                         final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final NEDVelocity oldVelocity,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final Angle oldLatitude,
                         final Angle oldLongitude, final Distance oldHeight,
                         final NEDVelocity oldVelocity,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final NEDFrame oldFrame,
                         final double latitude, final double longitude,
                         final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final NEDFrame oldFrame,
                         final double latitude, final double longitude,
                         final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final NEDFrame oldFrame,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final NEDFrame oldFrame,
                         final Angle latitude, final Angle longitude,
                         final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final NEDPosition oldPosition,
                         final double oldVn, final double oldVe, final double oldVd,
                         final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final NEDPosition oldPosition,
                         final double oldVn, final double oldVe, final double oldVd,
                         final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final NEDPosition oldPosition,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final NEDPosition oldPosition,
                         final Speed oldVn, final Speed oldVe, final Speed oldVd,
                         final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final NEDPosition oldPosition,
                         final NEDVelocity oldVelocity, final NEDPosition position,
                         final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, oldVelocity, position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final NEDPosition oldPosition,
                         final NEDVelocity oldVelocity, final NEDPosition position,
                         final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, oldVelocity, position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final double timeInterval, final NEDFrame oldFrame,
                         final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public void estimate(final Time timeInterval, final NEDFrame oldFrame,
                         final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final double oldLatitude,
                                            final double oldLongitude, final double oldHeight,
                                            final double oldVn, final double oldVe, final double oldVd,
                                            final double latitude, final double longitude,
                                            final double height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final double oldLatitude,
                                            final double oldLongitude, final double oldHeight,
                                            final double oldVn, final double oldVe, final double oldVd,
                                            final double latitude, final double longitude,
                                            final double height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final Angle oldLatitude,
                                            final Angle oldLongitude, final Distance oldHeight,
                                            final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final Angle oldLatitude,
                                            final Angle oldLongitude, final Distance oldHeight,
                                            final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final Angle oldLatitude,
                                            final Angle oldLongitude, final Distance oldHeight,
                                            final double oldVn, final double oldVe, final double oldVd,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final Angle oldLatitude,
                                            final Angle oldLongitude, final Distance oldHeight,
                                            final double oldVn, final double oldVe, final double oldVd,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final double oldLatitude,
                                            final double oldLongitude, final double oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final double latitude, final double longitude,
                                            final double height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final double oldLatitude,
                                            final double oldLongitude, final double oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final double latitude, final double longitude,
                                            final double height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final Angle oldLatitude,
                                            final Angle oldLongitude, final Distance oldHeight,
                                            final NEDVelocity oldVelocity,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final Angle oldLatitude,
                                            final Angle oldLongitude, final Distance oldHeight,
                                            final NEDVelocity oldVelocity, final Angle latitude,
                                            final Angle longitude, final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final NEDFrame oldFrame,
                                            final double latitude, final double longitude,
                                            final double height) {
        return estimateVelocityAndReturnNew(timeInterval, oldFrame, latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final NEDFrame oldFrame,
                                            final double latitude, final double longitude,
                                            final double height) {
        return estimateVelocityAndReturnNew(timeInterval, oldFrame, latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final NEDFrame oldFrame,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval, oldFrame,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final NEDFrame oldFrame,
                                            final Angle latitude, final Angle longitude,
                                            final Distance height) {
        return estimateVelocityAndReturnNew(timeInterval, oldFrame,
                latitude, longitude, height);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final NEDPosition oldPosition,
                                            final double oldVn, final double oldVe, final double oldVd,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final NEDPosition oldPosition,
                                            final double oldVn, final double oldVe, final double oldVd,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final NEDPosition oldPosition,
                                            final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final NEDPosition oldPosition,
                                            final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final NEDPosition oldPosition,
                                            final NEDVelocity oldVelocity,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldPosition, oldVelocity,
                position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final NEDPosition oldPosition,
                                            final NEDVelocity oldVelocity,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldPosition, oldVelocity,
                position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final double timeInterval, final NEDFrame oldFrame,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldFrame, position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public NEDVelocity estimateAndReturnNew(final Time timeInterval, final NEDFrame oldFrame,
                                            final NEDPosition position) {
        return estimateVelocityAndReturnNew(timeInterval, oldFrame, position);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double latitude, final double longitude,
                                        final double height, final NEDVelocity result) {

        if (timeInterval <= 0.0) {
            throw new IllegalArgumentException();
        }

        // Calculate meridian and transverse radii of curvature
        final RadiiOfCurvature oldRadii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(oldLatitude);
        final double oldRn = oldRadii.getRn();
        final double oldRe = oldRadii.getRe();

        final RadiiOfCurvature radii = RadiiOfCurvatureEstimator
                .estimateRadiiOfCurvatureAndReturnNew(latitude);
        final double re = radii.getRe();

        // Differentiate latitude, longitude, and height
        final double latRate = (latitude - oldLatitude) / timeInterval;
        final double longRate = (longitude - oldLongitude) / timeInterval;
        final double heightRate = (height - oldHeight) / timeInterval;

        // Derive the current velocity using (5.56)
        final double vn = (oldRn + height) * (2.0 * latRate - oldVn / (oldRn + oldHeight));
        final double ve = ((re + height) * Math.cos(latitude))
                * (2.0 * longRate - oldVe / ((oldRe + oldHeight) * Math.cos(oldLatitude)));
        final double vd = -2.0 * heightRate - oldVd;

        result.setCoordinates(vn, ve, vd);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final double latitude, final double longitude,
                                        final double height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, convertAngleToDouble(oldLatitude),
                convertAngleToDouble(oldLongitude), convertDistanceToDouble(oldHeight),
                convertSpeedToDouble(oldVn), convertSpeedToDouble(oldVe),
                convertSpeedToDouble(oldVd), convertAngleToDouble(latitude),
                convertAngleToDouble(longitude), convertDistanceToDouble(height), result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, convertAngleToDouble(oldLatitude),
                convertAngleToDouble(oldLongitude), convertDistanceToDouble(oldHeight),
                oldVn, oldVe, oldVd, convertAngleToDouble(latitude),
                convertAngleToDouble(longitude), convertDistanceToDouble(height), result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final double latitude, final double longitude,
                                        final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final double oldLatitude,
                                        final double oldLongitude, final double oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final double latitude, final double longitude,
                                        final double height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                latitude, longitude, height, result);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final Angle oldLatitude,
                                        final Angle oldLongitude, final Distance oldHeight,
                                        final NEDVelocity oldVelocity,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final NEDFrame oldFrame,
                                        final double latitude, final double longitude,
                                        final double height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame.getLatitude(), oldFrame.getLongitude(),
                oldFrame.getHeight(), oldFrame.getVn(), oldFrame.getVe(), oldFrame.getVd(),
                latitude, longitude, height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final NEDFrame oldFrame,
                                        final double latitude, final double longitude,
                                        final double height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldFrame, latitude, longitude, height,
                result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final NEDFrame oldFrame,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, convertAngleToDouble(latitude),
                convertAngleToDouble(longitude), convertDistanceToDouble(height), result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final NEDFrame oldFrame,
                                        final Angle latitude, final Angle longitude,
                                        final Distance height, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldFrame, latitude, longitude,
                height, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final NEDPosition oldPosition,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition.getLatitude(), oldPosition.getLongitude(),
                oldPosition.getHeight(), oldVn, oldVe, oldVd, position.getLatitude(),
                position.getLongitude(), position.getHeight(), result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final NEDPosition oldPosition,
                                        final double oldVn, final double oldVe, final double oldVd,
                                        final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldPosition, oldVn, oldVe, oldVd,
                position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final NEDPosition oldPosition,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition, convertSpeedToDouble(oldVn),
                convertSpeedToDouble(oldVe), convertSpeedToDouble(oldVd), position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final NEDPosition oldPosition,
                                        final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                        final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldPosition, oldVn, oldVe, oldVd,
                position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final NEDPosition oldPosition,
                                        final NEDVelocity oldVelocity,
                                        final NEDPosition position,
                                        final NEDVelocity result) {
        estimateVelocity(timeInterval, oldPosition,
                oldVelocity.getVn(), oldVelocity.getVe(), oldVelocity.getVd(),
                position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final NEDPosition oldPosition,
                                        final NEDVelocity oldVelocity,
                                        final NEDPosition position,
                                        final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldPosition, oldVelocity,
                position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final double timeInterval, final NEDFrame oldFrame,
                                        final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(timeInterval, oldFrame, position.getLatitude(), position.getLongitude(),
                position.getHeight(), result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param result       instance where updated velocity with respect the Earth, resolved
     *                     about north, east and down and expressed in meters per second (m/s) will
     *                     be stored.
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static void estimateVelocity(final Time timeInterval, final NEDFrame oldFrame,
                                        final NEDPosition position, final NEDVelocity result) {
        estimateVelocity(convertTimeToDouble(timeInterval), oldFrame, position, result);
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final double oldLatitude,
                                                           final double oldLongitude, final double oldHeight,
                                                           final double oldVn, final double oldVe, final double oldVd,
                                                           final double latitude, final double longitude,
                                                           final double height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final double oldLatitude,
                                                           final double oldLongitude, final double oldHeight,
                                                           final double oldVn, final double oldVe, final double oldVd,
                                                           final double latitude, final double longitude,
                                                           final double height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
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
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final Angle oldLatitude,
                                                           final Angle oldLongitude, final Distance oldHeight,
                                                           final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
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
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis.
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis.
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final Angle oldLatitude,
                                                           final Angle oldLongitude, final Distance oldHeight,
                                                           final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
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
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final Angle oldLatitude,
                                                           final Angle oldLongitude, final Distance oldHeight,
                                                           final double oldVn, final double oldVe, final double oldVd,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
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
     * @param oldVn        previous velocity of body with respect the Earth, resolved about
     *                     north-axis and expressed in meters per second (m/s).
     * @param oldVe        previous velocity of body with respect the Earth, resolved about
     *                     east-axis and expressed in meters per second (m/s).
     * @param oldVd        previous velocity of body with respect the Earth, resolved about
     *                     down-axis and expressed in meters per second (m/s).
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final Angle oldLatitude,
                                                           final Angle oldLongitude, final Distance oldHeight,
                                                           final double oldVn, final double oldVe, final double oldVd,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final double oldLatitude,
                                                           final double oldLongitude, final double oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final double latitude, final double longitude,
                                                           final double height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldLatitude  previous latitude expressed in radians (rad).
     * @param oldLongitude previous longitude expressed in radians (rad).
     * @param oldHeight    previous height expressed in meters (m).
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final double oldLatitude,
                                                           final double oldLongitude, final double oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final double latitude, final double longitude,
                                                           final double height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height, result);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final Angle oldLatitude,
                                                           final Angle oldLongitude, final Distance oldHeight,
                                                           final NEDVelocity oldVelocity,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
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
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final Angle oldLatitude,
                                                           final Angle oldLongitude, final Distance oldHeight,
                                                           final NEDVelocity oldVelocity, final Angle latitude,
                                                           final Angle longitude, final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final NEDFrame oldFrame,
                                                           final double latitude, final double longitude,
                                                           final double height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude expressed in radians (rad).
     * @param longitude    current longitude expressed in radians (rad).
     * @param height       current height expressed in meters (m).
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final NEDFrame oldFrame,
                                                           final double latitude, final double longitude,
                                                           final double height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final NEDFrame oldFrame,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param latitude     current latitude.
     * @param longitude    current longitude.
     * @param height       current height.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final NEDFrame oldFrame,
                                                           final Angle latitude, final Angle longitude,
                                                           final Distance height) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final NEDPosition oldPosition,
                                                           final double oldVn, final double oldVe, final double oldVd,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final NEDPosition oldPosition,
                                                           final double oldVn, final double oldVe, final double oldVd,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final NEDPosition oldPosition,
                                                           final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
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
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final NEDPosition oldPosition,
                                                           final Speed oldVn, final Speed oldVe, final Speed oldVd,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position,
                result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final NEDPosition oldPosition,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldPosition, oldVelocity, position, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @param oldVelocity  previous velocity of body with respect the Earth, resolved
     *                     about north, east and down axes and expressed in meters per
     *                     second (m/s).
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final NEDPosition oldPosition,
                                                           final NEDVelocity oldVelocity,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldPosition, oldVelocity, position,
                result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final double timeInterval, final NEDFrame oldFrame,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldFrame, position, result);
        return result;
    }

    /**
     * Estimates velocity from curvilinear position changes and taking into account previous
     * velocity respect NED frame.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous frame containing position and velocity of body with
     *                     respect Earth and resolved about north, east and down axes.
     * @param position     current body position with respect the Earth, resolved about
     *                     north, east and down axes.
     * @return estimated updated velocity with respect the Earth, resolved about north,
     * east and down and expressed in meters per second (m/s).
     * @throws IllegalArgumentException if provided time interval is negative or zero.
     */
    public static NEDVelocity estimateVelocityAndReturnNew(final Time timeInterval, final NEDFrame oldFrame,
                                                           final NEDPosition position) {
        final NEDVelocity result = new NEDVelocity();
        estimateVelocity(timeInterval, oldFrame, position, result);
        return result;
    }

    /**
     * Converts provided time instance to seconds (s).
     *
     * @param time time instance to be converted.
     * @return provided time value expressed in seconds.
     */
    private static double convertTimeToDouble(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
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
