/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;

import java.util.Objects;

/**
 * Extension of FrameBodyKinematics containing standard deviations of measured specific force
 * and angular rates included on measured body kinematics (accelerometer + gyroscope) besides
 * the corresponding frame (position, orientation and velocity) where the kinematics
 * measurement was made.
 */
public class StandardDeviationFrameBodyKinematics extends FrameBodyKinematics {

    /**
     * Standard deviation of measured specific force expressed in meters per squared
     * second (m/s^2).
     */
    private double mSpecificForceStandardDeviation;

    /**
     * Standard deviation of measured angular rate expressed in radians per second (rad/s).
     */
    private double mAngularRateStandardDeviation;

    /**
     * Constructor.
     */
    public StandardDeviationFrameBodyKinematics() {
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     */
    public StandardDeviationFrameBodyKinematics(final BodyKinematics kinematics) {
        super(kinematics);
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     */
    public StandardDeviationFrameBodyKinematics(final ECEFFrame frame) {
        super(frame);
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it will be
     *              converted to its corresponding ECEF frame.
     */
    public StandardDeviationFrameBodyKinematics(final NEDFrame frame) {
        super(frame);
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval expressed in seconds (s) between IMU measurements
     *                     used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(final double timeInterval) {
        super(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval between IMU measurements used to obtain
     *                     current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(final Time timeInterval) {
        super(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame) {
        super(frame, previousFrame);
    }

    /**
     * Constructor.
     *
     * @param frame         current NED frame associated to measurement. Internally it will be
     *                      converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it will be
     *                      converted to its corresponding ECEF frame.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame) {
        super(frame, previousFrame);
    }

    /**
     * Constructor.
     *
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval expressed in seconds (s) between IMU measurements
     *                      used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final double timeInterval) {
        super(frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final Time timeInterval) {
        super(frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval expressed in seconds (s) between IMU measurements
     *                      used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final double timeInterval) {
        super(frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final Time timeInterval) {
        super(frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     * @param frame      ECEF frame associated to measurement.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame) {
        super(kinematics, frame);
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     * @param frame      NED frame associated to measurement. Internally it will be
     *                   converted to its corresponding ECEF frame.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame) {
        super(kinematics, frame);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame) {
        super(kinematics, frame, previousFrame);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame) {
        super(kinematics, frame, previousFrame);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval expressed in seconds (s) between IMU
     *                      measurements used to obtain current frame and previous
     *                      frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final double timeInterval) {
        super(kinematics, frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final Time timeInterval) {
        super(kinematics, frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval expressed in seconds (s) between IMU
     *                      measurements used to obtain current frame and previous
     *                      frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final double timeInterval) {
        super(kinematics, frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final Time timeInterval) {
        super(kinematics, frame, previousFrame, timeInterval);
    }

    /**
     * Constructor.
     *
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if any provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final double timeInterval, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if any provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final Time timeInterval, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame, previousFrame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame, previousFrame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final double timeInterval, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval between IMU measurements
     *                                       used to obtain current frame and previous
     *                                       frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final Time timeInterval, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final double timeInterval, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final Time timeInterval, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          ECEF frame associated to measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          NED frame associated to measurements.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final double timeInterval,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final Time timeInterval,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final double timeInterval,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force expressed in meters per squared
     *                                       second (m/s^2).
     * @param angularRateStandardDeviation   standard deviation of measured angular rate
     *                                       expressed in radians per second (rad/s).
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final Time timeInterval,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval);
        setSpecificForceStandardDeviation(specificForceStandardDeviation);
        setAngularRateStandardDeviation(angularRateStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to measurement.
     *                                       Internally it will be converted to its
     *                                       corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if any provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final double timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(timeInterval, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if any provided value is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final Time timeInterval, final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(timeInterval, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, previousFrame, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous END frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, previousFrame, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final double timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated ot
     *                                       measurement.
     * @param timeInterval                   time interval between IMU measurements
     *                                       used to obtain current frame and previous
     *                                       frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final ECEFFrame frame, final ECEFFrame previousFrame,
            final Time timeInterval, final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to tis corresponding ECEF frame.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final double timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final NEDFrame frame, final NEDFrame previousFrame,
            final Time timeInterval, final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          ECEF frame associated to measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          NED frame associated to measurements.
     *                                       Internally it will be converted to tis
     *                                       corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated ot
     *                                       measurement. Internally it will be converted
     *                                       to its corresponding ECEF frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either specific force standard deviation or
     *                                  angular rate standard deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final double timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current ECEF frame associated to
     *                                       measurement.
     * @param previousFrame                  previous ECEF frame associated to
     *                                       measurement.
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final ECEFFrame frame,
            final ECEFFrame previousFrame, final Time timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval expressed in seconds (s)
     *                                       between IMU measurements used to obtain
     *                                       current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final double timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param kinematics                     current body kinematics measurement.
     * @param frame                          current NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param previousFrame                  previous NED frame associated to
     *                                       measurement. Internally it will be
     *                                       converted to its corresponding ECEF frame.
     * @param timeInterval                   time interval between IMU measurements used
     *                                       to obtain current frame and previous frame.
     * @param specificForceStandardDeviation standard deviation of measured specific
     *                                       force.
     * @param angularRateStandardDeviation   standard deviation of measured angular
     *                                       rate.
     * @throws IllegalArgumentException if either time interval, specific force
     *                                  standard deviation or angular rate standard
     *                                  deviation is negative.
     */
    public StandardDeviationFrameBodyKinematics(
            final BodyKinematics kinematics, final NEDFrame frame,
            final NEDFrame previousFrame, final Time timeInterval,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, frame, previousFrame, timeInterval,
                convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Copy constructor.
     *
     * @param input instance to copy data from.
     */
    public StandardDeviationFrameBodyKinematics(
            final StandardDeviationFrameBodyKinematics input) {
        copyFrom(input);
    }

    /**
     * Gets standard deviation of measured specific force expressed in meters per squared
     * second (m/s^2).
     *
     * @return standard deviation of measured specific force.
     */
    public double getSpecificForceStandardDeviation() {
        return mSpecificForceStandardDeviation;
    }

    /**
     * Sets standard deviation of measured specific force expressed in meters per squared
     * second (m/s^2).
     *
     * @param specificForceStandardDeviation standard deviation of measured specific force.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setSpecificForceStandardDeviation(
            final double specificForceStandardDeviation) {
        if (specificForceStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mSpecificForceStandardDeviation = specificForceStandardDeviation;
    }

    /**
     * Gets standard deviation of measured specific force.
     *
     * @return standard deviation of measured specific force.
     */
    public Acceleration getSpecificForceStandardDeviationAsAcceleration() {
        return new Acceleration(mSpecificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets standard deviation of measured specific force.
     *
     * @param result instance where standard deviation of measured specific force will be
     *               stored.
     */
    public void getSpecificForceStandardDeviationAsAcceleration(
            final Acceleration result) {
        result.setValue(mSpecificForceStandardDeviation);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets standard deviation of measured specific force.
     *
     * @param specificForceStandardDeviation standard deviation of measured specific force.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setSpecificForceStandardDeviation(
            final Acceleration specificForceStandardDeviation) {
        setSpecificForceStandardDeviation(convertAcceleration(
                specificForceStandardDeviation));
    }

    /**
     * Gets standard deviation of measured angular rate expressed in radians per second (rad/s).
     *
     * @return standard deviation of measured angular rate.
     */
    public double getAngularRateStandardDeviation() {
        return mAngularRateStandardDeviation;
    }

    /**
     * Sets standard deviation of measured angular rate expressed in radians per second (rad/s).
     *
     * @param angularRateStandardDeviation standard deviation of measured angular rate.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setAngularRateStandardDeviation(final double angularRateStandardDeviation) {
        if (angularRateStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mAngularRateStandardDeviation = angularRateStandardDeviation;
    }

    /**
     * Gets standard deviation of measured angular rate.
     *
     * @return standard deviation of measured angular rate.
     */
    public AngularSpeed getAngularRateStandardDeviationAsAngularSpeed() {
        return new AngularSpeed(mAngularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets standard deviation of measured angular rate.
     *
     * @param result instance where standard deviation of measured angular rate will be
     *               stored.
     */
    public void getAngularRateStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        result.setValue(mAngularRateStandardDeviation);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets standard deviation of measured angular rate.
     *
     * @param angularRateStandardDeviation standard deviation of measured angular rate.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setAngularRateStandardDeviation(
            final AngularSpeed angularRateStandardDeviation) {
        setAngularRateStandardDeviation(convertAngularSpeed(
                angularRateStandardDeviation));
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     *
     */
    public void copyFrom(final StandardDeviationFrameBodyKinematics input) {
        super.copyFrom(input);

        mSpecificForceStandardDeviation = input.mSpecificForceStandardDeviation;
        mAngularRateStandardDeviation = input.mAngularRateStandardDeviation;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final StandardDeviationFrameBodyKinematics output) {
        super.copyTo(output);

        output.mSpecificForceStandardDeviation = mSpecificForceStandardDeviation;
        output.mAngularRateStandardDeviation = mAngularRateStandardDeviation;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), mSpecificForceStandardDeviation, mAngularRateStandardDeviation);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(StandardDeviationFrameBodyKinematics other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     * @param other instance to be compared.
     * @param threshold maximum allowed difference between values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(StandardDeviationFrameBodyKinematics other,
                          final double threshold) {
        return super.equals(other, threshold) &&
                (mSpecificForceStandardDeviation - other.mSpecificForceStandardDeviation) <= threshold
                && (mAngularRateStandardDeviation - other.mAngularRateStandardDeviation) <= threshold;
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between kinematics and frame values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    @Override
    public boolean equals(final FrameBodyKinematics other, final double threshold) {
        if (this == other) {
            return true;
        }
        if (other == null || getClass() != other.getClass()) {
            return false;
        }
        return super.equals(other, threshold);
    }

    /**
     * Checks if provided object is a StandardDeviationFrameBodyKinematics instance
     * having exactly the same contents as this instance.
     *
     * @param obj object to be compared.
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
        final StandardDeviationFrameBodyKinematics other =
                (StandardDeviationFrameBodyKinematics) obj;
        return equals(other);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final StandardDeviationFrameBodyKinematics result =
                (StandardDeviationFrameBodyKinematics) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Converts provided acceleration to meters per squared second (m/s^2).
     *
     * @param acceleration instance to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts provided angular speed to radians per second (rad/s).
     *
     * @param angularSpeed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return AngularSpeedConverter.convert(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }
}
