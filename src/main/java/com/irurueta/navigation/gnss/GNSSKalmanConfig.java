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
package com.irurueta.navigation.gnss;

import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains GNSS Kalman filter configuration parameters (usually obtained through calibration) to
 * determine the system noise covariance matrix.
 */
@SuppressWarnings("WeakerAccess")
public class GNSSKalmanConfig implements Serializable, Cloneable {

    /**
     * Initial position uncertainty per axis expressed in meters (m).
     */
    private double mInitialPositionUncertainty;

    /**
     * Initial velocity uncertainty per axis expressed in meters per second (m/s).
     */
    private double mInitialVelocityUncertainty;

    /**
     * Initial clock offset uncertainty per axis expressed in meters (m).
     */
    private double mInitialClockOffsetUncertainty;

    /**
     * Initial clock drift uncertainty per axis expressed in meters per second (m/s).
     */
    private double mInitialClockDriftUncertainty;

    /**
     * Acceleration PSD (Power Spectral Density) per axis expressed in (m^2/s^3).
     */
    private double mAccelerationPSD;

    /**
     * Receiver clock frequency-drift PSD (Power Spectral Density) expressed in
     * (m^2/s^3).
     */
    private double mClockFrequencyPSD;

    /**
     * Receiver clock phase-drift PSD (Power Spectral Density) expressed in (m^2/s).
     */
    private double mClockPhasePSD;

    /**
     * Pseudo-range measurement noise SD (Standard Deviation) expressed in meters (m).
     */
    private double mPseudoRangeSD;

    /**
     * Pseudo-range rate measurement noise SD (Standard Deviation) expressed in meters
     * per second (m/s).
     */
    private double mRangeRateSD;

    /**
     * Constructor.
     */
    public GNSSKalmanConfig() {
    }

    /**
     * Constructor.
     *
     * @param initialPositionUncertainty    initial position uncertainty per axis expressed in
     *                                      meters (m).
     * @param initialVelocityUncertainty    initial velocity uncertainty per axis expressed in
     *                                      meters per second (m/s).
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis expressed in
     *                                      meters (m).
     * @param initialClockDriftUncertainty  initial clock drift uncertainty per axis expressed in
     *                                      meters per second (m/s).
     * @param accelerationPSD               acceleration PSD (Power Spectral Density) per axis
     *                                      expressed in (m^2/s^3).
     * @param clockFrequencyPSD             receiver clock frequency-drift PSD (Power Spectral
     *                                      Density) expressed in (m^2/s^3).
     * @param clockPhasePSD                 receiver clock phase-drift PSD (Power Spectral Density)
     *                                      expressed in (m^2/s).
     * @param pseudoRangeSD                 pseudo-range measurement noise SD (Standard Deviation)
     *                                      expressed in meters (m).
     * @param rangeRateSD                   pseudo-range measurement noise SD (Standard Deviation)
     *                                      expressed in meters per second (m/s).
     */
    public GNSSKalmanConfig(final double initialPositionUncertainty,
                            final double initialVelocityUncertainty,
                            final double initialClockOffsetUncertainty,
                            final double initialClockDriftUncertainty,
                            final double accelerationPSD, final double clockFrequencyPSD,
                            final double clockPhasePSD, final double pseudoRangeSD,
                            final double rangeRateSD) {
        setValues(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);
    }

    /**
     * Constructor.
     *
     * @param initialPositionUncertainty    initial position uncertainty per axis.
     * @param initialVelocityUncertainty    initial velocity uncertainty per axis.
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis.
     * @param initialClockDriftUncertainty  initial clock drift uncertainty per axis.
     * @param accelerationPSD               acceleration PSD (Power Spectral Density) per axis
     *                                      expressed in (m^2/s^3).
     * @param clockFrequencyPSD             receiver clock frequency-drift PSD (Power Spectral
     *                                      Density) expressed in (m^2/s^3).
     * @param clockPhasePSD                 receiver clock phase-drift PSD (Power Spectral Density)
     *                                      expressed in (m^2/s).
     * @param pseudoRangeSD                 pseudo-range measurement noise SD (Standard Deviation)
     *                                      expressed in meters (m).
     * @param rangeRateSD                   pseudo-range measurement noise SD (Standard Deviation)
     *                                      expressed in meters per second (m/s).
     */
    public GNSSKalmanConfig(final Distance initialPositionUncertainty,
                            final Speed initialVelocityUncertainty,
                            final Distance initialClockOffsetUncertainty,
                            final Speed initialClockDriftUncertainty,
                            final double accelerationPSD, final double clockFrequencyPSD,
                            final double clockPhasePSD, final Distance pseudoRangeSD,
                            final Speed rangeRateSD) {
        setValues(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty,
                accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public GNSSKalmanConfig(final GNSSKalmanConfig input) {
        copyFrom(input);
    }

    /**
     * Gets initial position uncertainty per axis expressed in meters (m).
     *
     * @return initial position uncertainty per axis.
     */
    public double getInitialPositionUncertainty() {
        return mInitialPositionUncertainty;
    }

    /**
     * Sets initial position uncertainty per axis expressed in meters (m).
     *
     * @param initialPositionUncertainty initial position uncertainty per axis.
     */
    public void setInitialPositionUncertainty(final double initialPositionUncertainty) {
        mInitialPositionUncertainty = initialPositionUncertainty;
    }

    /**
     * Gets initial position uncertainty per axis.
     *
     * @param result instance where initial position uncertainty per axis will be stored.
     */
    public void getDistanceInitialPositionUncertainty(final Distance result) {
        result.setValue(mInitialPositionUncertainty);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial position uncertainty per axis.
     *
     * @return initial position uncertainty per axis.
     */
    public Distance getDistanceInitialPositionUncertainty() {
        return new Distance(mInitialPositionUncertainty, DistanceUnit.METER);
    }

    /**
     * Sets initial position uncertainty per axis.
     *
     * @param initialPositionUncertainty initial position uncertainty per axis.
     */
    public void setInitialPositionUncertainty(final Distance initialPositionUncertainty) {
        mInitialPositionUncertainty = DistanceConverter.convert(
                initialPositionUncertainty.getValue().doubleValue(), initialPositionUncertainty.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets initial velocity uncertainty per axis expressed in meters per second (m/s).
     *
     * @return initial velocity uncertainty per axis.
     */
    public double getInitialVelocityUncertainty() {
        return mInitialVelocityUncertainty;
    }

    /**
     * Sets initial velocity uncertainty per axis expressed in meters per second (m/s).
     *
     * @param initialVelocityUncertainty initial velocity uncertainty per axis.
     */
    public void setInitialVelocityUncertainty(final double initialVelocityUncertainty) {
        mInitialVelocityUncertainty = initialVelocityUncertainty;
    }

    /**
     * Gets initial velocity uncertainty per axis.
     *
     * @param result instance where initial velocity uncertainty per axis will be stored.
     */
    public void getSpeedInitialVelocityUncertainty(final Speed result) {
        result.setValue(mInitialVelocityUncertainty);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial velocity uncertainty per axis.
     *
     * @return initial velocity uncertainty per axis.
     */
    public Speed getSpeedInitialVelocityUncertainty() {
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
                initialVelocityUncertainty.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial clock offset uncertainty per axis expressed in meters (m).
     *
     * @return initial clock offset uncertainty per axis.
     */
    public double getInitialClockOffsetUncertainty() {
        return mInitialClockOffsetUncertainty;
    }

    /**
     * Sets initial clock offset uncertainty per axis expressed in meters (m).
     *
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis.
     */
    public void setInitialClockOffsetUncertainty(final double initialClockOffsetUncertainty) {
        mInitialClockOffsetUncertainty = initialClockOffsetUncertainty;
    }

    /**
     * Gets initial clock offset uncertainty per axis.
     *
     * @param result instance where initial clock offset uncertainty per axis will be stored.
     */
    public void getDistanceInitialClockOffsetUncertainty(final Distance result) {
        result.setValue(mInitialClockOffsetUncertainty);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial clock offset uncertainty per axis.
     *
     * @return initial clock offset uncertainty per axis.
     */
    public Distance getDistanceInitialClockOffsetUncertainty() {
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
     * @return initial clock drift uncertainty per axis.
     */
    public double getInitialClockDriftUncertainty() {
        return mInitialClockDriftUncertainty;
    }

    /**
     * Sets initial clock drift uncertainty per axis expressed in meters per second (m/s).
     *
     * @param initialClockDriftUncertainty initial clock drift uncertainty per axis.
     */
    public void setInitialClockDriftUncertainty(final double initialClockDriftUncertainty) {
        mInitialClockDriftUncertainty = initialClockDriftUncertainty;
    }

    /**
     * Gets initial clock drift uncertainty per axis.
     *
     * @param result instance where initial clock drift uncertainty per axis will be stored.
     */
    public void getSpeedInitialClockDriftUncertainty(final Speed result) {
        result.setValue(mInitialClockDriftUncertainty);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial clock drift uncertainty per axis.
     *
     * @return initial clock drift uncertainty per axis.
     */
    public Speed getSpeedInitialClockDriftUncertainty() {
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
                initialClockDriftUncertainty.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets acceleration PSD (Power Spectral Density) per axis expressed in (m^2/s^3).
     *
     * @return acceleration PSD.
     */
    public double getAccelerationPSD() {
        return mAccelerationPSD;
    }

    /**
     * Sets acceleration PSD (Power Spectral Density) per axis expressed in (m^2/s^3).
     *
     * @param accelerationPSD acceleration PSD.
     */
    public void setAccelerationPSD(final double accelerationPSD) {
        mAccelerationPSD = accelerationPSD;
    }

    /**
     * Gets receiver clock frequency-drift PSD (Power Spectral Density) expressed in
     * (m^2/s^3).
     *
     * @return receiver clock frequency-drift PSD.
     */
    public double getClockFrequencyPSD() {
        return mClockFrequencyPSD;
    }

    /**
     * Sets receiver clock frequency-drift PSD (Power Spectral Density) expressed in
     * (m^2/s^3).
     *
     * @param clockFrequencyPSD receiver clock frequency-drift PSD.
     */
    public void setClockFrequencyPSD(final double clockFrequencyPSD) {
        mClockFrequencyPSD = clockFrequencyPSD;
    }

    /**
     * Gets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * (m^2/s).
     *
     * @return receiver clock phase-drift PSD.
     */
    public double getClockPhasePSD() {
        return mClockPhasePSD;
    }

    /**
     * Sets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * (m^2/s).
     *
     * @param clockPhasePSD receiver clock phase-drift PSD.
     */
    public void setClockPhasePSD(final double clockPhasePSD) {
        mClockPhasePSD = clockPhasePSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation) expressed in meters (m).
     *
     * @return pseudo-range measurement noise SD.
     */
    public double getPseudoRangeSD() {
        return mPseudoRangeSD;
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation) expressed in meters (m).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final double pseudoRangeSD) {
        mPseudoRangeSD = pseudoRangeSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range measurement noise SD will be stored.
     */
    public void getDistancePseudoRangeSD(final Distance result) {
        result.setValue(mPseudoRangeSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range measurement noise SD.
     */
    public Distance getDistancePseudoRangeSD() {
        return new Distance(mPseudoRangeSD, DistanceUnit.METER);
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final Distance pseudoRangeSD) {
        mPseudoRangeSD = DistanceConverter.convert(
                pseudoRangeSD.getValue().doubleValue(), pseudoRangeSD.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range rate measurement SD (Standard Deviation) expressed in meters
     * per second (m/s).
     *
     * @return pseudo-range rate measurement SD.
     */
    public double getRangeRateSD() {
        return mRangeRateSD;
    }

    /**
     * Sets pseudo-range rate measurement SD (Standard Deviation) expressed in meters
     * per second (m/s).
     *
     * @param rangeRateSD pseudo-range rate measurement SD.
     */
    public void setRangeRateSD(final double rangeRateSD) {
        mRangeRateSD = rangeRateSD;
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range rate measurement noise SD will be
     *               stored.
     */
    public void getSpeedRangeRateSD(final Speed result) {
        result.setValue(mRangeRateSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range rate measurement noise SD.
     */
    public Speed getSpeedRangeRateSD() {
        return new Speed(mRangeRateSD, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param rangeRateSD pseudo-range rate measurement noise SD.
     */
    public void setRangeRateSD(final Speed rangeRateSD) {
        mRangeRateSD = SpeedConverter.convert(rangeRateSD.getValue().doubleValue(),
                rangeRateSD.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets configuration parameters.
     *
     * @param initialPositionUncertainty    initial position uncertainty per axis expressed in
     *                                      meters (m).
     * @param initialVelocityUncertainty    initial velocity uncertainty per axis expressed in
     *                                      meters per second (m/s).
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis expressed in
     *                                      meters (m).
     * @param initialClockDriftUncertainty  initial clock drift uncertainty per axis expressed in
     *                                      meters per second (m/s).
     * @param accelerationPSD               acceleration PSD (Power Spectral Density) per axis
     *                                      expressed in (m^2/s^3).
     * @param clockFrequencyPSD             receiver clock frequency-drift PSD (Power Spectral
     *                                      Density) expressed in (m^2/s^3).
     * @param clockPhasePSD                 receiver clock phase-drift PSD (Power Spectral Density)
     *                                      expressed in (m^2/s).
     * @param pseudoRangeSD                 pseudo-range measurement noise SD (Standard Deviation)
     *                                      expressed in meters (m).
     * @param rangeRateSD                   pseudo-range measurement noise SD (Standard Deviation)
     *                                      expressed in meters per second (m/s).
     */
    public void setValues(final double initialPositionUncertainty,
                          final double initialVelocityUncertainty,
                          final double initialClockOffsetUncertainty,
                          final double initialClockDriftUncertainty,
                          final double accelerationPSD, final double clockFrequencyPSD,
                          final double clockPhasePSD, final double pseudoRangeSD,
                          final double rangeRateSD) {
        mInitialPositionUncertainty = initialPositionUncertainty;
        mInitialVelocityUncertainty = initialVelocityUncertainty;
        mInitialClockOffsetUncertainty = initialClockOffsetUncertainty;
        mInitialClockDriftUncertainty = initialClockDriftUncertainty;
        mAccelerationPSD = accelerationPSD;
        mClockFrequencyPSD = clockFrequencyPSD;
        mClockPhasePSD = clockPhasePSD;
        mPseudoRangeSD = pseudoRangeSD;
        mRangeRateSD = rangeRateSD;
    }

    /**
     * Sets configuration parameters.
     *
     * @param initialPositionUncertainty    initial position uncertainty per axis.
     * @param initialVelocityUncertainty    initial velocity uncertainty per axis.
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis.
     * @param initialClockDriftUncertainty  initial clock drift uncertainty per axis.
     * @param accelerationPSD               acceleration PSD (Power Spectral Density) per axis
     *                                      expressed in (m^2/s^3).
     * @param clockFrequencyPSD             receiver clock frequency-drift PSD (Power Spectral
     *                                      Density) expressed in (m^2/s^3).
     * @param clockPhasePSD                 receiver clock phase-drift PSD (Power Spectral Density)
     *                                      expressed in (m^2/s).
     * @param pseudoRangeSD                 pseudo-range measurement noise SD (Standard Deviation).
     * @param rangeRateSD                   pseudo-range measurement noise SD (Standard Deviation).
     */
    public void setValues(final Distance initialPositionUncertainty,
                          final Speed initialVelocityUncertainty,
                          final Distance initialClockOffsetUncertainty,
                          final Speed initialClockDriftUncertainty,
                          final double accelerationPSD, final double clockFrequencyPSD,
                          final double clockPhasePSD, final Distance pseudoRangeSD,
                          final Speed rangeRateSD) {
        setInitialPositionUncertainty(initialPositionUncertainty);
        setInitialVelocityUncertainty(initialVelocityUncertainty);
        setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);
        setInitialClockDriftUncertainty(initialClockDriftUncertainty);
        mAccelerationPSD = accelerationPSD;
        mClockFrequencyPSD = clockFrequencyPSD;
        mClockPhasePSD = clockPhasePSD;
        setPseudoRangeSD(pseudoRangeSD);
        setRangeRateSD(rangeRateSD);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final GNSSKalmanConfig output) {
        output.mInitialPositionUncertainty = mInitialPositionUncertainty;
        output.mInitialVelocityUncertainty = mInitialVelocityUncertainty;
        output.mInitialClockOffsetUncertainty = mInitialClockOffsetUncertainty;
        output.mInitialClockDriftUncertainty = mInitialClockDriftUncertainty;
        output.mAccelerationPSD = mAccelerationPSD;
        output.mClockFrequencyPSD = mClockFrequencyPSD;
        output.mClockPhasePSD = mClockPhasePSD;
        output.mPseudoRangeSD = mPseudoRangeSD;
        output.mRangeRateSD = mRangeRateSD;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final GNSSKalmanConfig input) {
        mInitialPositionUncertainty = input.mInitialPositionUncertainty;
        mInitialVelocityUncertainty = input.mInitialVelocityUncertainty;
        mInitialClockOffsetUncertainty = input.mInitialClockOffsetUncertainty;
        mInitialClockDriftUncertainty = input.mInitialClockDriftUncertainty;
        mAccelerationPSD = input.mAccelerationPSD;
        mClockFrequencyPSD = input.mClockFrequencyPSD;
        mClockPhasePSD = input.mClockPhasePSD;
        mPseudoRangeSD = input.mPseudoRangeSD;
        mRangeRateSD = input.mRangeRateSD;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mInitialPositionUncertainty, mInitialVelocityUncertainty,
                mInitialClockOffsetUncertainty, mInitialClockDriftUncertainty,
                mAccelerationPSD, mClockFrequencyPSD, mClockPhasePSD,
                mPseudoRangeSD, mRangeRateSD);
    }

    /**
     * Checks if provided object is a GNSSKalmanConfig having exactly the same contents
     * as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof GNSSKalmanConfig)) {
            return false;
        }

        final GNSSKalmanConfig other = (GNSSKalmanConfig) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final GNSSKalmanConfig other) {
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
    public boolean equals(final GNSSKalmanConfig other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mInitialPositionUncertainty - other.mInitialPositionUncertainty) <= threshold
                && Math.abs(mInitialVelocityUncertainty - other.mInitialVelocityUncertainty) <= threshold
                && Math.abs(mInitialClockOffsetUncertainty - other.mInitialClockOffsetUncertainty) <= threshold
                && Math.abs(mInitialClockDriftUncertainty - other.mInitialClockDriftUncertainty) <= threshold
                && Math.abs(mAccelerationPSD - other.mAccelerationPSD) <= threshold
                && Math.abs(mClockFrequencyPSD - other.mClockFrequencyPSD) <= threshold
                && Math.abs(mClockPhasePSD - other.mClockPhasePSD) <= threshold
                && Math.abs(mPseudoRangeSD - other.mPseudoRangeSD) <= threshold
                && Math.abs(mRangeRateSD - other.mRangeRateSD) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     */
    @SuppressWarnings({"CloneDoesntDeclareCloneNotSupportedException", "MethodDoesntCallSuperMethod"})
    @Override
    protected Object clone() {
        return new GNSSKalmanConfig(this);
    }
}
