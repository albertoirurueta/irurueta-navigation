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
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a body kinematics measurement (accelerometer + gyroscope) along with the
 * corresponding timestamp when measure was made.
 * Notice that timestamp does not need to be absolute.
 * Usually timestamps are used in sequences of measurements of body kinematics, where
 * the first measurement can have any timestamp value (e.g. zero), and hence the subsequent
 * measurements will have timestamps relative to the first one.
 */
public class TimedBodyKinematics implements Serializable, Cloneable {

    /**
     * Current body kinematics measurement. Contains accelerometer and gyroscope measurements.
     */
    private BodyKinematics mKinematics;

    /**
     * Timestamp value expressed in seconds.
     */
    private double mTimestampSeconds;

    /**
     * Constructor.
     */
    public TimedBodyKinematics() {
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     */
    public TimedBodyKinematics(final BodyKinematics kinematics) {
        mKinematics = kinematics;
    }

    /**
     * Constructor.
     *
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public TimedBodyKinematics(final double timestampSeconds) {
        mTimestampSeconds = timestampSeconds;
    }

    /**
     * Constructor.
     *
     * @param timestamp timestamp value.
     */
    public TimedBodyKinematics(final Time timestamp) {
        mTimestampSeconds = convertTime(timestamp);
    }

    /**
     * Constructor.
     *
     * @param kinematics       current body kinematics measurement.
     * @param timestampSeconds timestamp value expressed in seconds.
     */
    public TimedBodyKinematics(final BodyKinematics kinematics,
                               final double timestampSeconds) {
        this(kinematics);
        mTimestampSeconds = timestampSeconds;
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     * @param timestamp  timestamp value.
     */
    public TimedBodyKinematics(final BodyKinematics kinematics,
                               final Time timestamp) {
        this(timestamp);
        mKinematics = kinematics;
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public TimedBodyKinematics(final TimedBodyKinematics input) {
        copyFrom(input);
    }

    /**
     * Gets current body kinematics measurement. Contains accelerometer and gyroscope
     * measurements.
     *
     * @return current body kinematics measurement.
     */
    public BodyKinematics getKinematics() {
        return mKinematics;
    }

    /**
     * Sets current body kinematics measurement. Contains accelerometer and gyroscope
     * measurements.
     *
     * @param kinematics current body kinematics measurement to be set.
     */
    public void setKinematics(final BodyKinematics kinematics) {
        mKinematics = kinematics;
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
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final TimedBodyKinematics input) {
        if (input.mKinematics != null) {
            if (mKinematics == null) {
                mKinematics = new BodyKinematics(input.mKinematics);
            } else {
                mKinematics.copyFrom(input.mKinematics);
            }
        } else {
            mKinematics = null;
        }

        mTimestampSeconds = input.mTimestampSeconds;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final TimedBodyKinematics output) {
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
        return Objects.hash(mKinematics, mTimestampSeconds);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final TimedBodyKinematics other) {
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
    public boolean equals(final TimedBodyKinematics other, final double threshold) {
        if (other == null) {
            return false;
        }

        return ((other.mKinematics == null && mKinematics == null)
                || (mKinematics != null && mKinematics.equals(other.mKinematics, threshold)))
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
        final TimedBodyKinematics other = (TimedBodyKinematics) obj;
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
        final TimedBodyKinematics result = (TimedBodyKinematics) super.clone();
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
