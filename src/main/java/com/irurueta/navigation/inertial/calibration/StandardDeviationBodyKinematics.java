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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a body kinematics measurement (accelerometer + gyroscope) along with
 * the corresponding standard deviations of measured specific force and angular
 * rates.
 */
public class StandardDeviationBodyKinematics implements Serializable, Cloneable {

    /**
     * Current body kinematics measurement. Contains accelerometer and gyroscope measurements.
     */
    private BodyKinematics mKinematics;

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
    public StandardDeviationBodyKinematics() {
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     */
    public StandardDeviationBodyKinematics(final BodyKinematics kinematics) {
        mKinematics = kinematics;
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
    public StandardDeviationBodyKinematics(
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
    public StandardDeviationBodyKinematics(
            final BodyKinematics kinematics,
            final double specificForceStandardDeviation,
            final double angularRateStandardDeviation) {
        this(kinematics);
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
    public StandardDeviationBodyKinematics(
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
    public StandardDeviationBodyKinematics(
            final BodyKinematics kinematics,
            final Acceleration specificForceStandardDeviation,
            final AngularSpeed angularRateStandardDeviation) {
        this(kinematics, convertAcceleration(specificForceStandardDeviation),
                convertAngularSpeed(angularRateStandardDeviation));
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public StandardDeviationBodyKinematics(final StandardDeviationBodyKinematics input) {
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
     */
    public void copyFrom(final StandardDeviationBodyKinematics input) {
        if (input.mKinematics != null) {
            if (mKinematics == null) {
                mKinematics = new BodyKinematics(input.mKinematics);
            } else {
                mKinematics.copyFrom(input.mKinematics);
            }
        } else {
            mKinematics = null;
        }

        mSpecificForceStandardDeviation = input.mSpecificForceStandardDeviation;
        mAngularRateStandardDeviation = input.mAngularRateStandardDeviation;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final StandardDeviationBodyKinematics output) {
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
        return Objects.hash(mKinematics, mSpecificForceStandardDeviation, mAngularRateStandardDeviation);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final StandardDeviationBodyKinematics other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between kinematics and standard deviation
     *                  values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final StandardDeviationBodyKinematics other, final double threshold) {
        if (other == null) {
            return false;
        }

        return ((other.mKinematics == null && mKinematics == null)
                || (mKinematics != null && mKinematics.equals(other.mKinematics, threshold)))
                && (mSpecificForceStandardDeviation - other.mSpecificForceStandardDeviation) <= threshold
                && (mAngularRateStandardDeviation - other.mAngularRateStandardDeviation) <= threshold;
    }

    /**
     * Checks if provided object is a StandardDeviationBodyKinematics instance having exactly the
     * same contents as this instance.
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
        final StandardDeviationBodyKinematics other = (StandardDeviationBodyKinematics) obj;
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
        final StandardDeviationBodyKinematics result = (StandardDeviationBodyKinematics) super.clone();
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
