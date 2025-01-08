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
public class GNSSKalmanConfig implements Serializable, Cloneable {

    /**
     * Initial position uncertainty per axis expressed in meters (m).
     */
    private double initialPositionUncertainty;

    /**
     * Initial velocity uncertainty per axis expressed in meters per second (m/s).
     */
    private double initialVelocityUncertainty;

    /**
     * Initial clock offset uncertainty per axis expressed in meters (m).
     */
    private double initialClockOffsetUncertainty;

    /**
     * Initial clock drift uncertainty per axis expressed in meters per second (m/s).
     */
    private double initialClockDriftUncertainty;

    /**
     * Acceleration PSD (Power Spectral Density) per axis expressed in (m^2/s^3).
     */
    private double accelerationPSD;

    /**
     * Receiver clock frequency-drift PSD (Power Spectral Density) expressed in
     * (m^2/s^3).
     */
    private double clockFrequencyPSD;

    /**
     * Receiver clock phase-drift PSD (Power Spectral Density) expressed in (m^2/s).
     */
    private double clockPhasePSD;

    /**
     * Pseudo-range measurement noise SD (Standard Deviation) expressed in meters (m).
     */
    private double pseudoRangeSD;

    /**
     * Pseudo-range rate measurement noise SD (Standard Deviation) expressed in meters
     * per second (m/s).
     */
    private double rangeRateSD;

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
        setValues(initialPositionUncertainty, initialVelocityUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
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
        setValues(initialPositionUncertainty, initialVelocityUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
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
        return initialPositionUncertainty;
    }

    /**
     * Sets initial position uncertainty per axis expressed in meters (m).
     *
     * @param initialPositionUncertainty initial position uncertainty per axis.
     */
    public void setInitialPositionUncertainty(final double initialPositionUncertainty) {
        this.initialPositionUncertainty = initialPositionUncertainty;
    }

    /**
     * Gets initial position uncertainty per axis.
     *
     * @param result instance where initial position uncertainty per axis will be stored.
     */
    public void getDistanceInitialPositionUncertainty(final Distance result) {
        result.setValue(initialPositionUncertainty);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial position uncertainty per axis.
     *
     * @return initial position uncertainty per axis.
     */
    public Distance getDistanceInitialPositionUncertainty() {
        return new Distance(initialPositionUncertainty, DistanceUnit.METER);
    }

    /**
     * Sets initial position uncertainty per axis.
     *
     * @param initialPositionUncertainty initial position uncertainty per axis.
     */
    public void setInitialPositionUncertainty(final Distance initialPositionUncertainty) {
        this.initialPositionUncertainty = DistanceConverter.convert(
                initialPositionUncertainty.getValue().doubleValue(), initialPositionUncertainty.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets initial velocity uncertainty per axis expressed in meters per second (m/s).
     *
     * @return initial velocity uncertainty per axis.
     */
    public double getInitialVelocityUncertainty() {
        return initialVelocityUncertainty;
    }

    /**
     * Sets initial velocity uncertainty per axis expressed in meters per second (m/s).
     *
     * @param initialVelocityUncertainty initial velocity uncertainty per axis.
     */
    public void setInitialVelocityUncertainty(final double initialVelocityUncertainty) {
        this.initialVelocityUncertainty = initialVelocityUncertainty;
    }

    /**
     * Gets initial velocity uncertainty per axis.
     *
     * @param result instance where initial velocity uncertainty per axis will be stored.
     */
    public void getSpeedInitialVelocityUncertainty(final Speed result) {
        result.setValue(initialVelocityUncertainty);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial velocity uncertainty per axis.
     *
     * @return initial velocity uncertainty per axis.
     */
    public Speed getSpeedInitialVelocityUncertainty() {
        return new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets initial velocity uncertainty per axis.
     *
     * @param initialVelocityUncertainty initial velocity uncertainty per axis.
     */
    public void setInitialVelocityUncertainty(final Speed initialVelocityUncertainty) {
        this.initialVelocityUncertainty = SpeedConverter.convert(initialVelocityUncertainty.getValue().doubleValue(),
                initialVelocityUncertainty.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial clock offset uncertainty per axis expressed in meters (m).
     *
     * @return initial clock offset uncertainty per axis.
     */
    public double getInitialClockOffsetUncertainty() {
        return initialClockOffsetUncertainty;
    }

    /**
     * Sets initial clock offset uncertainty per axis expressed in meters (m).
     *
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis.
     */
    public void setInitialClockOffsetUncertainty(final double initialClockOffsetUncertainty) {
        this.initialClockOffsetUncertainty = initialClockOffsetUncertainty;
    }

    /**
     * Gets initial clock offset uncertainty per axis.
     *
     * @param result instance where initial clock offset uncertainty per axis will be stored.
     */
    public void getDistanceInitialClockOffsetUncertainty(final Distance result) {
        result.setValue(initialClockOffsetUncertainty);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial clock offset uncertainty per axis.
     *
     * @return initial clock offset uncertainty per axis.
     */
    public Distance getDistanceInitialClockOffsetUncertainty() {
        return new Distance(initialClockOffsetUncertainty, DistanceUnit.METER);
    }

    /**
     * Sets initial clock offset uncertainty per axis.
     *
     * @param initialClockOffsetUncertainty initial clock offset uncertainty per axis.
     */
    public void setInitialClockOffsetUncertainty(final Distance initialClockOffsetUncertainty) {
        this.initialClockOffsetUncertainty = DistanceConverter.convert(
                initialClockOffsetUncertainty.getValue().doubleValue(), initialClockOffsetUncertainty.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets initial clock drift uncertainty per axis expressed in meters per second (m/s).
     *
     * @return initial clock drift uncertainty per axis.
     */
    public double getInitialClockDriftUncertainty() {
        return initialClockDriftUncertainty;
    }

    /**
     * Sets initial clock drift uncertainty per axis expressed in meters per second (m/s).
     *
     * @param initialClockDriftUncertainty initial clock drift uncertainty per axis.
     */
    public void setInitialClockDriftUncertainty(final double initialClockDriftUncertainty) {
        this.initialClockDriftUncertainty = initialClockDriftUncertainty;
    }

    /**
     * Gets initial clock drift uncertainty per axis.
     *
     * @param result instance where initial clock drift uncertainty per axis will be stored.
     */
    public void getSpeedInitialClockDriftUncertainty(final Speed result) {
        result.setValue(initialClockDriftUncertainty);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial clock drift uncertainty per axis.
     *
     * @return initial clock drift uncertainty per axis.
     */
    public Speed getSpeedInitialClockDriftUncertainty() {
        return new Speed(initialClockDriftUncertainty, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets initial clock drift uncertainty per axis.
     *
     * @param initialClockDriftUncertainty initial clock drift uncertainty per axis.
     */
    public void setInitialClockDriftUncertainty(final Speed initialClockDriftUncertainty) {
        this.initialClockDriftUncertainty = SpeedConverter.convert(
                initialClockDriftUncertainty.getValue().doubleValue(), initialClockDriftUncertainty.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets acceleration PSD (Power Spectral Density) per axis expressed in (m^2/s^3).
     *
     * @return acceleration PSD.
     */
    public double getAccelerationPSD() {
        return accelerationPSD;
    }

    /**
     * Sets acceleration PSD (Power Spectral Density) per axis expressed in (m^2/s^3).
     *
     * @param accelerationPSD acceleration PSD.
     */
    public void setAccelerationPSD(final double accelerationPSD) {
        this.accelerationPSD = accelerationPSD;
    }

    /**
     * Gets receiver clock frequency-drift PSD (Power Spectral Density) expressed in
     * (m^2/s^3).
     *
     * @return receiver clock frequency-drift PSD.
     */
    public double getClockFrequencyPSD() {
        return clockFrequencyPSD;
    }

    /**
     * Sets receiver clock frequency-drift PSD (Power Spectral Density) expressed in
     * (m^2/s^3).
     *
     * @param clockFrequencyPSD receiver clock frequency-drift PSD.
     */
    public void setClockFrequencyPSD(final double clockFrequencyPSD) {
        this.clockFrequencyPSD = clockFrequencyPSD;
    }

    /**
     * Gets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * (m^2/s).
     *
     * @return receiver clock phase-drift PSD.
     */
    public double getClockPhasePSD() {
        return clockPhasePSD;
    }

    /**
     * Sets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * (m^2/s).
     *
     * @param clockPhasePSD receiver clock phase-drift PSD.
     */
    public void setClockPhasePSD(final double clockPhasePSD) {
        this.clockPhasePSD = clockPhasePSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation) expressed in meters (m).
     *
     * @return pseudo-range measurement noise SD.
     */
    public double getPseudoRangeSD() {
        return pseudoRangeSD;
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation) expressed in meters (m).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final double pseudoRangeSD) {
        this.pseudoRangeSD = pseudoRangeSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range measurement noise SD will be stored.
     */
    public void getDistancePseudoRangeSD(final Distance result) {
        result.setValue(pseudoRangeSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range measurement noise SD.
     */
    public Distance getDistancePseudoRangeSD() {
        return new Distance(pseudoRangeSD, DistanceUnit.METER);
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final Distance pseudoRangeSD) {
        this.pseudoRangeSD = DistanceConverter.convert(pseudoRangeSD.getValue().doubleValue(), pseudoRangeSD.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range rate measurement SD (Standard Deviation) expressed in meters
     * per second (m/s).
     *
     * @return pseudo-range rate measurement SD.
     */
    public double getRangeRateSD() {
        return rangeRateSD;
    }

    /**
     * Sets pseudo-range rate measurement SD (Standard Deviation) expressed in meters
     * per second (m/s).
     *
     * @param rangeRateSD pseudo-range rate measurement SD.
     */
    public void setRangeRateSD(final double rangeRateSD) {
        this.rangeRateSD = rangeRateSD;
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range rate measurement noise SD will be
     *               stored.
     */
    public void getSpeedRangeRateSD(final Speed result) {
        result.setValue(rangeRateSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range rate measurement noise SD.
     */
    public Speed getSpeedRangeRateSD() {
        return new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param rangeRateSD pseudo-range rate measurement noise SD.
     */
    public void setRangeRateSD(final Speed rangeRateSD) {
        this.rangeRateSD = SpeedConverter.convert(rangeRateSD.getValue().doubleValue(), rangeRateSD.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
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
        this.initialPositionUncertainty = initialPositionUncertainty;
        this.initialVelocityUncertainty = initialVelocityUncertainty;
        this.initialClockOffsetUncertainty = initialClockOffsetUncertainty;
        this.initialClockDriftUncertainty = initialClockDriftUncertainty;
        this.accelerationPSD = accelerationPSD;
        this.clockFrequencyPSD = clockFrequencyPSD;
        this.clockPhasePSD = clockPhasePSD;
        this.pseudoRangeSD = pseudoRangeSD;
        this.rangeRateSD = rangeRateSD;
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
    public void setValues(final Distance initialPositionUncertainty, final Speed initialVelocityUncertainty,
                          final Distance initialClockOffsetUncertainty, final Speed initialClockDriftUncertainty,
                          final double accelerationPSD, final double clockFrequencyPSD, final double clockPhasePSD,
                          final Distance pseudoRangeSD, final Speed rangeRateSD) {
        setInitialPositionUncertainty(initialPositionUncertainty);
        setInitialVelocityUncertainty(initialVelocityUncertainty);
        setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);
        setInitialClockDriftUncertainty(initialClockDriftUncertainty);
        this.accelerationPSD = accelerationPSD;
        this.clockFrequencyPSD = clockFrequencyPSD;
        this.clockPhasePSD = clockPhasePSD;
        setPseudoRangeSD(pseudoRangeSD);
        setRangeRateSD(rangeRateSD);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final GNSSKalmanConfig output) {
        output.initialPositionUncertainty = initialPositionUncertainty;
        output.initialVelocityUncertainty = initialVelocityUncertainty;
        output.initialClockOffsetUncertainty = initialClockOffsetUncertainty;
        output.initialClockDriftUncertainty = initialClockDriftUncertainty;
        output.accelerationPSD = accelerationPSD;
        output.clockFrequencyPSD = clockFrequencyPSD;
        output.clockPhasePSD = clockPhasePSD;
        output.pseudoRangeSD = pseudoRangeSD;
        output.rangeRateSD = rangeRateSD;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final GNSSKalmanConfig input) {
        initialPositionUncertainty = input.initialPositionUncertainty;
        initialVelocityUncertainty = input.initialVelocityUncertainty;
        initialClockOffsetUncertainty = input.initialClockOffsetUncertainty;
        initialClockDriftUncertainty = input.initialClockDriftUncertainty;
        accelerationPSD = input.accelerationPSD;
        clockFrequencyPSD = input.clockFrequencyPSD;
        clockPhasePSD = input.clockPhasePSD;
        pseudoRangeSD = input.pseudoRangeSD;
        rangeRateSD = input.rangeRateSD;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(initialPositionUncertainty, initialVelocityUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);
    }

    /**
     * Checks if provided object is a GNSSKalmanConfig having exactly the same contents
     * as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == this) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }

        final var other = (GNSSKalmanConfig) obj;
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

        return Math.abs(initialPositionUncertainty - other.initialPositionUncertainty) <= threshold
                && Math.abs(initialVelocityUncertainty - other.initialVelocityUncertainty) <= threshold
                && Math.abs(initialClockOffsetUncertainty - other.initialClockOffsetUncertainty) <= threshold
                && Math.abs(initialClockDriftUncertainty - other.initialClockDriftUncertainty) <= threshold
                && Math.abs(accelerationPSD - other.accelerationPSD) <= threshold
                && Math.abs(clockFrequencyPSD - other.clockFrequencyPSD) <= threshold
                && Math.abs(clockPhasePSD - other.clockPhasePSD) <= threshold
                && Math.abs(pseudoRangeSD - other.pseudoRangeSD) <= threshold
                && Math.abs(rangeRateSD - other.rangeRateSD) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (GNSSKalmanConfig)super.clone();
        copyTo(result);
        return result;
    }
}
