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
package com.irurueta.navigation.inertial;

import com.irurueta.units.*;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains INS/GNS Tightly Coupled Kalman filter configuration parameters (usually
 * obtained through calibration) to determine the system noise covariance matrix.
 */
public class INSTightlyCoupledKalmanConfig implements Serializable, Cloneable {

    /**
     * Initial attitude uncertainty per axis expressed in radians (rad).
     */
    private double mInitialAttitudeUncertainty;

    /**
     * Initial velocity uncertainty per axis expressed in meters per second (m/s).
     */
    private double mInitialVelocityUncertainty;

    /**
     * Initial position uncertainty per axis expressed in meters (m).
     */
    private double mInitialPositionUncertainty;

    /**
     * Initial acceleration bias uncertainty expressed in meters per squared second (m/s^2).
     */
    private double mInitialAccelerationBiasUncertainty;

    /**
     * Initial gyroscope bias uncertainty expressed in radians per second (rad/s).
     */
    private double mInitialGyroscopeBiasUncertainty;

    /**
     * Initial clock offset uncertainty per axis expressed in meters (m).
     */
    private double mInitialClockOffsetUncertainty;

    /**
     * Initial clock drift uncertainty per axis expressed in meters per second (m/s).
     */
    private double mInitialClockDriftUncertainty;

    /**
     * Constructor.
     */
    public INSTightlyCoupledKalmanConfig() {
    }

    /**
     * Constructor.
     *
     * @param initialAttitudeUncertainty         initial attitude uncertainty per axis
     *                                           expressed in radians (rad).
     * @param initialVelocityUncertainty         initial velocity uncertainty per axis
     *                                           expressed in meters per second (m/s).
     * @param initialPositionUncertainty         initial position uncertainty per axis
     *                                           expressed in meters (m).
     * @param initialAccelerationBiasUncertainty initial acceleration bias uncertainty
     *                                           expressed in meters per squared second (m/s^2).
     * @param initialGyroscopeBiasUncertainty    initial gyroscope bias uncertainty
     *                                           expressed in radians per second (rad/s).
     * @param initialClockOffsetUncertainty      initial clock offset uncertainty per axis
     *                                           expressed in meters (m).
     * @param initialClockDriftUncertainty       initial clock drift uncertainty per axis
     *                                           expressed in meters per second (m/s).
     */
    public INSTightlyCoupledKalmanConfig(final double initialAttitudeUncertainty,
                                         final double initialVelocityUncertainty,
                                         final double initialPositionUncertainty,
                                         final double initialAccelerationBiasUncertainty,
                                         final double initialGyroscopeBiasUncertainty,
                                         final double initialClockOffsetUncertainty,
                                         final double initialClockDriftUncertainty) {
        setValues(initialAttitudeUncertainty,
                initialVelocityUncertainty,
                initialPositionUncertainty,
                initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty,
                initialClockDriftUncertainty);
    }

    /**
     * Constructor.
     *
     * @param initialAttitudeUncertainty         initial attitude uncertainty per axis.
     * @param initialVelocityUncertainty         initial velocity uncertainty per axis.
     * @param initialPositionUncertainty         initial position uncertainty per axis.
     * @param initialAccelerationBiasUncertainty initial acceleration bias uncertainty.
     * @param initialGyroscopeBiasUncertainty    initial gyroscope bias uncertainty.
     * @param initialClockOffsetUncertainty      initial clock offset uncertainty per axis.
     * @param initialClockDriftUncertainty       initial clock drift uncertainty per axis.
     */
    public INSTightlyCoupledKalmanConfig(final Angle initialAttitudeUncertainty,
                                         final Speed initialVelocityUncertainty,
                                         final Distance initialPositionUncertainty,
                                         final Acceleration initialAccelerationBiasUncertainty,
                                         final AngularSpeed initialGyroscopeBiasUncertainty,
                                         final Distance initialClockOffsetUncertainty,
                                         final Speed initialClockDriftUncertainty) {
        setValues(initialAttitudeUncertainty,
                initialVelocityUncertainty,
                initialPositionUncertainty,
                initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty,
                initialClockDriftUncertainty);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public INSTightlyCoupledKalmanConfig(final INSTightlyCoupledKalmanConfig input) {
        copyFrom(input);
    }

    /**
     * Gets initial attitude uncertainty per axis expressed in radians (rad).
     *
     * @return initial attitude uncertainty per axis expressed in radians (rad).
     */
    public double getInitialAttitudeUncertainty() {
        return mInitialAttitudeUncertainty;
    }

    /**
     * Sets initial attitude uncertainty per axis expressed in radians (rad).
     *
     * @param initialAttitudeUncertainty initial attitude uncertainty per axis expressed
     *                                   in radians (rad).
     */
    public void setInitialAttitudeUncertainty(final double initialAttitudeUncertainty) {
        mInitialAttitudeUncertainty = initialAttitudeUncertainty;
    }

    /**
     * Gets initial attitude uncertainty per axis.
     *
     * @param result instance where initial attitude uncertainty per axis will be stored.
     */
    public void getInitialAttitudeUncertaintyAngle(final Angle result) {
        result.setValue(mInitialAttitudeUncertainty);
        result.setUnit(AngleUnit.RADIANS);
    }

    /**
     * Gets initial attitude uncertainty per axis.
     *
     * @return initial attitude uncertainty per axis.
     */
    public Angle getInitialAttitudeUncertaintyAngle() {
        return new Angle(mInitialAttitudeUncertainty, AngleUnit.RADIANS);
    }

    /**
     * Sets initial attitude uncertainty per axis.
     *
     * @param initialAttitudeUncertainty initial attitude uncertainty per axis.
     */
    public void setInitialAttitudeUncertainty(final Angle initialAttitudeUncertainty) {
        mInitialAttitudeUncertainty = AngleConverter.convert(
                initialAttitudeUncertainty.getValue().doubleValue(),
                initialAttitudeUncertainty.getUnit(), AngleUnit.RADIANS);
    }

    /**
     * Gets initial velocity uncertainty per axis expressed in meters per second (m/s).
     *
     * @return initial velocity uncertainty per axis expressed in meters per second (m/s).
     */
    public double getInitialVelocityUncertainty() {
        return mInitialVelocityUncertainty;
    }

    /**
     * Sets initial velocity uncertainty per axis expressed in meters per second (m/s).
     *
     * @param initialVelocityUncertainty initial velocity uncertainty per axis expressed
     *                                   in meters per second (m/s).
     */
    public void setInitialVelocityUncertainty(final double initialVelocityUncertainty) {
        mInitialVelocityUncertainty = initialVelocityUncertainty;
    }

    /**
     * Gets initial velocity uncertainty per axis.
     *
     * @param result instance where initial attitude uncertainty per axis will be stored.
     */
    public void getInitialVelocityUncertaintySpeed(final Speed result) {
        result.setValue(mInitialVelocityUncertainty);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial velocity uncertainty per axis.
     *
     * @return initial velocity uncertainty per axis.
     */
    public Speed getInitialVelocityUncertaintySpeed() {
        return new Speed(mInitialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets initial velocity uncertainty per axis.
     *
     * @param initialVelocityUncertainty initial velocity uncertainty per axis.
     */
    public void setInitialVelocityUncertainty(final Speed initialVelocityUncertainty) {
        mInitialVelocityUncertainty = SpeedConverter.convert(
                initialVelocityUncertainty.getValue().doubleValue(),
                initialVelocityUncertainty.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial position uncertainty per axis expressed in meters (m)
     *
     * @return initial position uncertainty per axis expressed in meters (m).
     */
    public double getInitialPositionUncertainty() {
        return mInitialPositionUncertainty;
    }

    /**
     * Sets initial position uncertainty per axis expressed in meters (m)
     *
     * @param initialPositionUncertainty initial position uncertainty per axis expressed
     *                                   in meters (m).
     */
    public void setInitialPositionUncertainty(final double initialPositionUncertainty) {
        mInitialPositionUncertainty = initialPositionUncertainty;
    }

    /**
     * Gets initial position uncertainty per axis.
     *
     * @param result instance where initial position uncertainty per axis will be stored.
     */
    public void getInitialPositionUncertaintyDistance(final Distance result) {
        result.setValue(mInitialPositionUncertainty);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial position uncertainty per axis.
     *
     * @return initial position uncertainty per axis.
     */
    public Distance getInitialPositionUncertaintyDistance() {
        return new Distance(mInitialPositionUncertainty, DistanceUnit.METER);
    }

    /**
     * Sets initial position uncertainty per axis.
     *
     * @param initialPositionUncertainty initial position uncertainty per axis.
     */
    public void setInitialPositionUncertainty(final Distance initialPositionUncertainty) {
        mInitialPositionUncertainty = DistanceConverter.convert(
                initialPositionUncertainty.getValue().doubleValue(),
                initialPositionUncertainty.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets initial acceleration bias uncertainty expressed in meters per squared second (m/s^2).
     *
     * @return initial acceleration bias uncertainty expressed in meters per squared second (m/s^2).
     */
    public double getInitialAccelerationBiasUncertainty() {
        return mInitialAccelerationBiasUncertainty;
    }

    /**
     * Sets initial acceleration bias uncertainty expressed in meters per squared second (m/s^2).
     *
     * @param initialAccelerationBiasUncertainty initial acceleration bias uncertainty expressed in
     *                                           meters per squared second (m/s^2).
     */
    public void setInitialAccelerationBiasUncertainty(
            final double initialAccelerationBiasUncertainty) {
        mInitialAccelerationBiasUncertainty = initialAccelerationBiasUncertainty;
    }

    /**
     * Gets initial acceleration bias uncertainty.
     *
     * @param result instance where initial acceleration bias uncertainty will be stored.
     */
    public void getInitialAccelerationBiasUncertaintyAcceleration(final Acceleration result) {
        result.setValue(mInitialAccelerationBiasUncertainty);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial acceleration bias uncertainty.
     *
     * @return initial acceleration bias uncertainty.
     */
    public Acceleration getInitialAccelerationBiasUncertaintyAcceleration() {
        return new Acceleration(mInitialAccelerationBiasUncertainty,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial acceleration bias uncertainty.
     *
     * @param initialAccelerationUncertainty initial acceleration bias uncertainty.
     */
    public void setInitialAccelerationBiasUncertainty(
            final Acceleration initialAccelerationUncertainty) {
        mInitialAccelerationBiasUncertainty = AccelerationConverter.convert(
                initialAccelerationUncertainty.getValue().doubleValue(),
                initialAccelerationUncertainty.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial gyroscope bias uncertainty expressed in radians per second (rad/s).
     *
     * @return initial gyroscope bias uncertainty expressed in radians per second (rad/s).
     */
    public double getInitialGyroscopeBiasUncertainty() {
        return mInitialGyroscopeBiasUncertainty;
    }

    /**
     * Sets initial gyroscope bias uncertainty expressed in radians per second (rad/s).
     *
     * @param initialGyroscopeBiasUncertainty initial gyroscope bias uncertainty expressed
     *                                        in radians per second (rad/s).
     */
    public void setInitialGyroscopeBiasUncertainty(
            final double initialGyroscopeBiasUncertainty) {
        mInitialGyroscopeBiasUncertainty = initialGyroscopeBiasUncertainty;
    }

    /**
     * Gets initial gyroscope bias uncertainty.
     *
     * @param result instance where initial gyroscope bias uncertainty will be stored.
     */
    public void getInitialGyroscopeBiasUncertaintyAngularSpeed(AngularSpeed result) {
        result.setValue(mInitialGyroscopeBiasUncertainty);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial gyroscope bias uncertainty.
     *
     * @return initial gyroscope bias uncertainty.
     */
    public AngularSpeed getInitialGyroscopeBiasUncertaintyAngularSpeed() {
        return new AngularSpeed(mInitialGyroscopeBiasUncertainty,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial gyroscope bias uncertainty.
     *
     * @param initialGyroscopeBiasUncertainty initial gyroscope bias uncertainty.
     */
    public void setInitialGyroscopeBiasUncertainty(
            final AngularSpeed initialGyroscopeBiasUncertainty) {
        mInitialGyroscopeBiasUncertainty = AngularSpeedConverter.convert(
                initialGyroscopeBiasUncertainty.getValue().doubleValue(),
                initialGyroscopeBiasUncertainty.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial clock offset uncertainty per axis expressed in meters (m).
     *
     * @return initial clock offset uncertainty per axis expressed in meters (m).
     */
    public double getInitialClockOffsetUncertainty() {
        return mInitialClockOffsetUncertainty;
    }

    /**
     * Sets initial clock offset uncertainty per axis expressed in meters (m).
     *
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per
     *                                      axis expressed in meters (m).
     */
    public void setInitialClockOffsetUncertainty(
            final double initialClockOffsetUncertainty) {
        mInitialClockOffsetUncertainty = initialClockOffsetUncertainty;
    }

    /**
     * Gets initial clock offset uncertainty per axis.
     *
     * @param result instance where initial clock offset uncertainty per axis will be stored.
     */
    public void getInitialClockOffsetUncertaintyDistance(final Distance result) {
        result.setValue(mInitialClockOffsetUncertainty);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial clock offset uncertainty per axis.
     *
     * @return initial clock offset uncertainty per axis.
     */
    public Distance getInitialClockOffsetUncertaintyDistance() {
        return new Distance(mInitialClockOffsetUncertainty, DistanceUnit.METER);
    }

    /**
     * Sets initial clock offset uncertainty per axis.
     *
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis.
     */
    public void setInitialClockOffsetUncertainty(final Distance initialClockOffsetUncertainty) {
        mInitialClockOffsetUncertainty = DistanceConverter.convert(
                initialClockOffsetUncertainty.getValue().doubleValue(),
                initialClockOffsetUncertainty.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets initial clock drift uncertainty per axis expressed in meters per second (m/s).
     *
     * @return initial clock drift uncertainty per axis expressed in meters per second (m/s).
     */
    public double getInitialClockDriftUncertainty() {
        return mInitialClockDriftUncertainty;
    }

    /**
     * Sets initial clock drift uncertainty per axis expressed in meters per second (m/s).
     *
     * @param initialClockDriftUncertainty initial clock drift uncertainty per axis expressed
     *                                     in meters per second (m/s).
     */
    public void setInitialClockDriftUncertainty(
            final double initialClockDriftUncertainty) {
        mInitialClockDriftUncertainty = initialClockDriftUncertainty;
    }

    /**
     * Gets initial clock drift uncertainty per axis.
     *
     * @param result instance where initial clock drift uncertainty per axis will be stored.
     */
    public void getInitialClockDriftUncertaintySpeed(final Speed result) {
        result.setValue(mInitialClockDriftUncertainty);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial clock drift uncertainty per axis.
     *
     * @return initial clock drift uncertainty per axis.
     */
    public Speed getInitialClockDriftUncertaintySpeed() {
        return new Speed(mInitialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets initial clock drift uncertainty per axis.
     *
     * @param initialClockDriftUncertainty initial clock drift uncertainty per axis.
     */
    public void setInitialClockDriftUncertainty(final Speed initialClockDriftUncertainty) {
        mInitialClockDriftUncertainty = SpeedConverter.convert(
                initialClockDriftUncertainty.getValue().doubleValue(),
                initialClockDriftUncertainty.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets configuration parameters.
     *
     * @param initialAttitudeUncertainty         initial attitude uncertainty per axis
     *                                           expressed in radians (rad).
     * @param initialVelocityUncertainty         initial velocity uncertainty per axis
     *                                           expressed in meters per second (m/s).
     * @param initialPositionUncertainty         initial position uncertainty per axis
     *                                           expressed in meters (m).
     * @param initialAccelerationBiasUncertainty initial acceleration bias uncertainty
     *                                           expressed in meters per squared second (m/s^2).
     * @param initialGyroscopeBiasUncertainty    initial gyroscope bias uncertainty
     *                                           expressed in radians per second (rad/s).
     * @param initialClockOffsetUncertainty      initial clock offset uncertainty per axis
     *                                           expressed in meters (m).
     * @param initialClockDriftUncertainty       initial clock drift uncertainty per axis
     *                                           expressed in meters per second (m/s).
     */
    public void setValues(final double initialAttitudeUncertainty,
                          final double initialVelocityUncertainty,
                          final double initialPositionUncertainty,
                          final double initialAccelerationBiasUncertainty,
                          final double initialGyroscopeBiasUncertainty,
                          final double initialClockOffsetUncertainty,
                          final double initialClockDriftUncertainty) {
        mInitialAttitudeUncertainty = initialAttitudeUncertainty;
        mInitialVelocityUncertainty = initialVelocityUncertainty;
        mInitialPositionUncertainty = initialPositionUncertainty;
        mInitialAccelerationBiasUncertainty = initialAccelerationBiasUncertainty;
        mInitialGyroscopeBiasUncertainty = initialGyroscopeBiasUncertainty;
        mInitialClockOffsetUncertainty = initialClockOffsetUncertainty;
        mInitialClockDriftUncertainty = initialClockDriftUncertainty;
    }

    /**
     * Sets configuration parameters.
     *
     * @param initialAttitudeUncertainty         initial attitude uncertainty per axis.
     * @param initialVelocityUncertainty         initial velocity uncertainty per axis.
     * @param initialPositionUncertainty         initial position uncertainty per axis.
     * @param initialAccelerationBiasUncertainty initial acceleration bias uncertainty.
     * @param initialGyroscopeBiasUncertainty    initial gyroscope bias uncertainty.
     * @param initialClockOffsetUncertainty      initial clock offset uncertainty per axis.
     * @param initialClockDriftUncertainty       initial clock drift uncertainty per axis.
     */
    public void setValues(final Angle initialAttitudeUncertainty,
                          final Speed initialVelocityUncertainty,
                          final Distance initialPositionUncertainty,
                          final Acceleration initialAccelerationBiasUncertainty,
                          final AngularSpeed initialGyroscopeBiasUncertainty,
                          final Distance initialClockOffsetUncertainty,
                          final Speed initialClockDriftUncertainty) {
        setInitialAttitudeUncertainty(initialAttitudeUncertainty);
        setInitialVelocityUncertainty(initialVelocityUncertainty);
        setInitialPositionUncertainty(initialPositionUncertainty);
        setInitialAccelerationBiasUncertainty(initialAccelerationBiasUncertainty);
        setInitialGyroscopeBiasUncertainty(initialGyroscopeBiasUncertainty);
        setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);
        setInitialClockDriftUncertainty(initialClockDriftUncertainty);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final INSTightlyCoupledKalmanConfig output) {
        output.mInitialAttitudeUncertainty = mInitialAttitudeUncertainty;
        output.mInitialVelocityUncertainty = mInitialVelocityUncertainty;
        output.mInitialPositionUncertainty = mInitialPositionUncertainty;
        output.mInitialAccelerationBiasUncertainty = mInitialAccelerationBiasUncertainty;
        output.mInitialGyroscopeBiasUncertainty = mInitialGyroscopeBiasUncertainty;
        output.mInitialClockOffsetUncertainty = mInitialClockOffsetUncertainty;
        output.mInitialClockDriftUncertainty = mInitialClockDriftUncertainty;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final INSTightlyCoupledKalmanConfig input) {
        mInitialAttitudeUncertainty = input.mInitialAttitudeUncertainty;
        mInitialVelocityUncertainty = input.mInitialVelocityUncertainty;
        mInitialPositionUncertainty = input.mInitialPositionUncertainty;
        mInitialAccelerationBiasUncertainty = input.mInitialAccelerationBiasUncertainty;
        mInitialGyroscopeBiasUncertainty = input.mInitialGyroscopeBiasUncertainty;
        mInitialClockOffsetUncertainty = input.mInitialClockOffsetUncertainty;
        mInitialClockDriftUncertainty = input.mInitialClockDriftUncertainty;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mInitialAttitudeUncertainty, mInitialVelocityUncertainty,
                mInitialPositionUncertainty, mInitialAccelerationBiasUncertainty,
                mInitialGyroscopeBiasUncertainty, mInitialClockOffsetUncertainty,
                mInitialClockDriftUncertainty);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param obj instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        INSTightlyCoupledKalmanConfig other = (INSTightlyCoupledKalmanConfig) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(INSTightlyCoupledKalmanConfig other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(INSTightlyCoupledKalmanConfig other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mInitialAttitudeUncertainty - other.mInitialAttitudeUncertainty) <= threshold
                && Math.abs(mInitialVelocityUncertainty - other.mInitialVelocityUncertainty) <= threshold
                && Math.abs(mInitialPositionUncertainty - other.mInitialPositionUncertainty) <= threshold
                && Math.abs(mInitialAccelerationBiasUncertainty - other.mInitialAccelerationBiasUncertainty) <= threshold
                && Math.abs(mInitialGyroscopeBiasUncertainty - other.mInitialGyroscopeBiasUncertainty) <= threshold
                && Math.abs(mInitialClockOffsetUncertainty - other.mInitialClockOffsetUncertainty) <= threshold
                && Math.abs(mInitialClockDriftUncertainty - other.mInitialClockDriftUncertainty) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final INSTightlyCoupledKalmanConfig result = (INSTightlyCoupledKalmanConfig) super.clone();
        copyTo(result);
        return result;
    }
}
