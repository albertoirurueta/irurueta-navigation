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
 * Contains GNSS configuration parameters (usually obtained through calibration) to
 * determine the system noise covariance matrix for the Kalman filter.
 */
@SuppressWarnings("WeakerAccess")
public class GNSSKalmanConfig implements Serializable, Cloneable {

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
     * @param accelerationPSD   acceleration PSD (Power Spectral Density) per axis
     *                          expressed in (m^2/s^3).
     * @param clockFrequencyPSD receiver clock frequency-drift PSD (Power Spectral
     *                          Density) expressed in (m^2/s^3).
     * @param clockPhasePSD     receiver clock phase-drift PSD (Power Spectral Density)
     *                          expressed in (m^2/s).
     * @param pseudoRangeSD     pseudo-range measurement noise SD (Standard Deviation)
     *                          expressed in meters (m).
     * @param rangeRateSD       pseudo-range measurement noise SD (Standard Deviation)
     *                          expressed in meters per second (m/s).
     */
    public GNSSKalmanConfig(final double accelerationPSD, final double clockFrequencyPSD,
                            final double clockPhasePSD, final double pseudoRangeSD,
                            final double rangeRateSD) {
        setValues(accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);
    }

    /**
     * Constructor.
     *
     * @param accelerationPSD   acceleration PSD (Power Spectral Density) per axis
     *                          expressed in (m^2/s^3).
     * @param clockFrequencyPSD receiver clock frequency-drift PSD (Power Spectral
     *                          Density) expressed in (m^2/s^3).
     * @param clockPhasePSD     receiver clock phase-drift PSD (Power Spectral Density)
     *                          expressed in (m^2/s).
     * @param pseudoRangeSD     pseudo-range measurement noise SD (Standard Deviation)
     *                          expressed in meters (m).
     * @param rangeRateSD       pseudo-range measurement noise SD (Standard Deviation)
     *                          expressed in meters per second (m/s).
     */
    public GNSSKalmanConfig(final double accelerationPSD, final double clockFrequencyPSD,
                            final double clockPhasePSD, final Distance pseudoRangeSD,
                            final Speed rangeRateSD) {
        setValues(accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
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
     * @param accelerationPSD   acceleration PSD (Power Spectral Density) per axis
     *                          expressed in (m^2/s^3).
     * @param clockFrequencyPSD receiver clock frequency-drift PSD (Power Spectral
     *                          Density) expressed in (m^2/s^3).
     * @param clockPhasePSD     receiver clock phase-drift PSD (Power Spectral Density)
     *                          expressed in (m^2/s).
     * @param pseudoRangeSD     pseudo-range measurement noise SD (Standard Deviation)
     *                          expressed in meters (m).
     * @param rangeRateSD       pseudo-range measurement noise SD (Standard Deviation)
     *                          expressed in meters per second (m/s).
     */
    public void setValues(final double accelerationPSD, final double clockFrequencyPSD,
                          final double clockPhasePSD, final double pseudoRangeSD,
                          final double rangeRateSD) {
        mAccelerationPSD = accelerationPSD;
        mClockFrequencyPSD = clockFrequencyPSD;
        mClockPhasePSD = clockPhasePSD;
        mPseudoRangeSD = pseudoRangeSD;
        mRangeRateSD = rangeRateSD;
    }

    /**
     * Sets configuration parameters.
     *
     * @param accelerationPSD   acceleration PSD (Power Spectral Density) per axis
     *                          expressed in (m^2/s^3).
     * @param clockFrequencyPSD receiver clock frequency-drift PSD (Power Spectral
     *                          Density) expressed in (m^2/s^3).
     * @param clockPhasePSD     receiver clock phase-drift PSD (Power Spectral Density)
     *                          expressed in (m^2/s).
     * @param pseudoRangeSD     pseudo-range measurement noise SD (Standard Deviation).
     * @param rangeRateSD       pseudo-range measurement noise SD (Standard Deviation).
     */
    public void setValues(final double accelerationPSD, final double clockFrequencyPSD,
                          final double clockPhasePSD, final Distance pseudoRangeSD,
                          final Speed rangeRateSD) {
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
        return Objects.hash(mAccelerationPSD, mClockFrequencyPSD, mClockPhasePSD,
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

        return Math.abs(mAccelerationPSD - other.mAccelerationPSD) <= threshold
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
