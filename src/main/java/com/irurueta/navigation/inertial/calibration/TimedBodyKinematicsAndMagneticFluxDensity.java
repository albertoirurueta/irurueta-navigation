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

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.Objects;

/**
 * Contains body kinematics describing the forces and angular rate applied to a body,
 * along with the sensed magnetic flux density resolved around body coordinates and the
 * corresponding timestamp when measure was made.
 * Notice that timestamp does not need to be absolute.
 * Usually timestamps are used in sequences of measurements of body kinematics, where
 * the first measurement can have any timestamp value (e.g. zero), and hence the subsequent
 * measurements will have timestamps relative to the first one.
 */
public class TimedBodyKinematicsAndMagneticFluxDensity extends BodyKinematicsAndMagneticFluxDensity {

    /**
     * Timestamp value expressed in seconds.
     */
    private double mTimestampSeconds;

    /**
     * Constructor.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity() {
        super();
    }

    /**
     * Constructor.
     *
     * @param kinematics body kinematics containing sensed specific force
     *                   and angular rate.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(final BodyKinematics kinematics) {
        super(kinematics);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity body magnetic flux density.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        super(magneticFluxDensity);
    }

    /**
     * Constructor.
     *
     * @param kinematics          body kinematics containing sensed specific force
     *                            and angular rate.
     * @param magneticFluxDensity body magnetic flux density.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity magneticFluxDensity) {
        super(kinematics, magneticFluxDensity);
    }

    /**
     * Constructor.
     *
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(final double timestampSeconds) {
        mTimestampSeconds = timestampSeconds;
    }

    /**
     * Constructor.
     *
     * @param timestamp timestamp value.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(final Time timestamp) {
        mTimestampSeconds = convertTime(timestamp);
    }

    /**
     * Constructor.
     *
     * @param kinematics       body kinematics containing sensed specific force
     *                         and angular rate.
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics,
            final double timestampSeconds) {
        super(kinematics);
        mTimestampSeconds = timestampSeconds;
    }

    /**
     * Constructor.
     *
     * @param kinematics body kinematics containing sensed specific force
     *                   and angular rate.
     * @param timestamp  timestamp value.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics,
            final Time timestamp) {
        super(kinematics);
        mTimestampSeconds = convertTime(timestamp);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity body magnetic flux density.
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final double timestampSeconds) {
        super(magneticFluxDensity);
        mTimestampSeconds = timestampSeconds;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity body magnetic flux density.
     * @param timestamp  timestamp value.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final Time timestamp) {
        super(magneticFluxDensity);
        mTimestampSeconds = convertTime(timestamp);
    }

    /**
     * Constructor.
     *
     * @param kinematics          body kinematics containing sensed specific force
     *                            and angular rate.
     * @param magneticFluxDensity body magnetic flux density.
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity magneticFluxDensity,
            final double timestampSeconds) {
        super(kinematics, magneticFluxDensity);
        mTimestampSeconds = timestampSeconds;
    }


    /**
     * Constructor.
     *
     * @param kinematics          body kinematics containing sensed specific force
     *                            and angular rate.
     * @param magneticFluxDensity body magnetic flux density.
     * @param timestamp  timestamp value.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics,
            final BodyMagneticFluxDensity magneticFluxDensity,
            final Time timestamp) {
        super(kinematics, magneticFluxDensity);
        mTimestampSeconds = convertTime(timestamp);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public TimedBodyKinematicsAndMagneticFluxDensity(
            final TimedBodyKinematicsAndMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets timestamp value expressed in seconds.
     *
     * @return timestamp value expressed in seconds.
     */
    public double getTimestampSeconds() {
        return mTimestampSeconds;
    }

    /**
     * Sets timestamp value expressed in seconds.
     *
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public void setTimestampSeconds(final double timestampSeconds) {
        mTimestampSeconds = timestampSeconds;
    }

    /**
     * Gets timestamp value.
     *
     * @return a new timestamp instance.
     */
    public Time getTimestamp() {
        return new Time(mTimestampSeconds, TimeUnit.SECOND);
    }

    /**
     * Gets timestamp value.
     *
     * @param result instance where result data will be stored.
     */
    public void getTimestamp(final Time result) {
        result.setValue(mTimestampSeconds);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets timestamp.
     *
     * @param timestamp timestamp to be set.
     */
    public void setTimestamp(final Time timestamp) {
        mTimestampSeconds = convertTime(timestamp);
    }

    /**
     * Gets a timed body kinematics instance containing current body kinematics
     * and timestamp.
     *
     * @return a timed body kinematics
     */
    public TimedBodyKinematics getTimedKinematics() {
        return new TimedBodyKinematics(getKinematics(), mTimestampSeconds);
    }

    /**
     * Gets a timed body kinematics instance containing current body kinematics
     * and timestamp.
     *
     * @param result instance where result will be stored.
     */
    public void getTimedKinematics(final TimedBodyKinematics result) {
        result.setKinematics(getKinematics());
        result.setTimestampSeconds(mTimestampSeconds);
    }

    /**
     * Sets data from provided timed body kinematics.
     *
     * @param timedKinematics timed body kinematics.
     */
    public void setTimedKinematics(final TimedBodyKinematics timedKinematics) {
        setKinematics(timedKinematics.getKinematics());
        mTimestampSeconds = timedKinematics.getTimestampSeconds();
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final TimedBodyKinematicsAndMagneticFluxDensity input) {
        super.copyFrom(input);
        mTimestampSeconds = input.mTimestampSeconds;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final TimedBodyKinematicsAndMagneticFluxDensity output) {
        output.copyFrom(this);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), mTimestampSeconds);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final TimedBodyKinematicsAndMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between kinematics and timestamp values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final TimedBodyKinematicsAndMagneticFluxDensity other, final double threshold) {
        return super.equals(other, threshold)
                && Math.abs(other.mTimestampSeconds - mTimestampSeconds) <= threshold;
    }

    /**
     * Checks if provided object is a TimedBodyKinematics instance having exactly the same
     * contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        final TimedBodyKinematicsAndMagneticFluxDensity other = (TimedBodyKinematicsAndMagneticFluxDensity) obj;
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
        final TimedBodyKinematicsAndMagneticFluxDensity result =
                (TimedBodyKinematicsAndMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Converts provided time instance to seconds.
     *
     * @param time timestamp to be converted.
     * @return converted value expressed in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
    }
}
