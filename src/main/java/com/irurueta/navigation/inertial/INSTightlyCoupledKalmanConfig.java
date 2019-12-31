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

import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains configuration parameters (usually obtained through calibration)
 * for INS/GNSS Loosely Coupled Kalman filter.
 */
public class INSTightlyCoupledKalmanConfig implements Serializable, Cloneable {

    /**
     * Gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     */
    private double mGyroNoisePSD;

    /**
     * Accelerometer noise PSD (Power Spectral Density) expressed in (m^2 * s^-3)
     */
    private double mAccelerometerNoisePSD;

    /**
     * Accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     */
    private double mAccelerometerBiasPSD;

    /**
     * Gyro bias random walk PSD (Power Spectral Density) expressed in (rad^2 * s^-3).
     */
    private double mGyroBiasPSD;

    /**
     * Receiver clock frequency-drift PSD (Power Spectral Density) expressed
     * in (m^2/s^3).
     */
    private double mClockFrequencyPSD;

    /**
     * Receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * squared meters per second (m^2/s).
     */
    private double mClockPhasePSD;

    /**
     * Pseudo-range measurement noise SD (Standard Deviation) expressed in
     * meters (m).
     */
    private double mPseudoRangeSD;

    /**
     * Pseudo-range rate measurement noise SD (Standard Deviation) expressed
     * in meters per second (m/s).
     */
    private double mRangeRateSD;

    /**
     * Constructor.
     */
    public INSTightlyCoupledKalmanConfig() {
    }

    /**
     * Constructor.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public INSTightlyCoupledKalmanConfig(final double gyroNoisePSD,
                                         final double accelerometerNoisePSD,
                                         final double accelerometerBiasPSD,
                                         final double gyroBiasPSD,
                                         final double clockFrequencyPSD,
                                         final double clockPhasePSD,
                                         final double pseudoRangeSD,
                                         final double rangeRateSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);
    }

    /**
     * Constructor.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelereometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public INSTightlyCoupledKalmanConfig(final double gyroNoisePSD,
                                         final double accelerometerNoisePSD,
                                         final double accelerometerBiasPSD,
                                         final double gyroBiasPSD,
                                         final double clockFrequencyPSD,
                                         final double clockPhasePSD,
                                         final Distance pseudoRangeSD,
                                         final Speed rangeRateSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                rangeRateSD);
    }

    /**
     * Gets gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     *
     * @return gyro noise PSD.
     */
    public double getGyroNoisePSD() {
        return mGyroNoisePSD;
    }

    /**
     * Sets gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     *
     * @param gyroNoisePSD gyro noise PSD.
     */
    public void setGyroNoisePSD(final double gyroNoisePSD) {
        mGyroNoisePSD = gyroNoisePSD;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD.
     */
    public double getAccelerometerNoisePSD() {
        return mAccelerometerNoisePSD;
    }

    /**
     * Sets accelerometer noise PSD (Power Spectral Density) expressed
     * in (m^2 * se^-3).
     *
     * @param accelerometerNoisePSD accelerometer noise PSD.
     */
    public void setAccelerometerNoisePSD(final double accelerometerNoisePSD) {
        mAccelerometerNoisePSD = accelerometerNoisePSD;
    }

    /**
     * Gets accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     *
     * @return accelerometer bias random walk PSD.
     */
    public double getAccelerometerBiasPSD() {
        return mAccelerometerBiasPSD;
    }

    /**
     * Sets accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     *
     * @param accelerometerBiasPSD accelerometer bias random walk PSD.
     */
    public void setAccelerometerBiasPSD(final double accelerometerBiasPSD) {
        mAccelerometerBiasPSD = accelerometerBiasPSD;
    }

    /**
     * Gets gyro bias random walk PSD (Power Spectral Density) expressed in
     * (rad^2 * s^-3).
     *
     * @return gyro bias random walk PSD.
     */
    public double getGyroBiasPSD() {
        return mGyroBiasPSD;
    }

    /**
     * Sets gyro bias random walk PSD (Power Spectral Density) expressed in
     * (rad^2 * s^-3).
     *
     * @param gyroBiasPSD gyro bias random walk PSD.
     */
    public void setGyroBiasPSD(final double gyroBiasPSD) {
        mGyroBiasPSD = gyroBiasPSD;
    }

    /**
     * Gets receiver clock frequency-drift PSD (Power Spectral Density) expressed
     * in (m^2/s^3).
     *
     * @return receiver clock frequency-drift PSD.
     */
    public double getClockFrequencyPSD() {
        return mClockFrequencyPSD;
    }

    /**
     * Sets receiver clock frequency-drift PSD (Power Spectral Density) expressed
     * in (m^2/s^3).
     *
     * @param clockFrequencyPSD clock frequency-drift PSD.
     */
    public void setClockFrequencyPSD(final double clockFrequencyPSD) {
        mClockFrequencyPSD = clockFrequencyPSD;
    }

    /**
     * Gets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * squared meters per second (m^2/s).
     *
     * @return receiver clock phase-drift PSD.
     */
    public double getClockPhasePSD() {
        return mClockPhasePSD;
    }

    /**
     * Sets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * squared meters per second (m^2/s).
     *
     * @param clockPhasePSD receiver clock phase-drift PSD.
     */
    public void setClockPhasePSD(final double clockPhasePSD) {
        mClockPhasePSD = clockPhasePSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation) expressed in
     * meters (m).
     *
     * @return pseudo-range measurement noise SD.
     */
    public double getPseudoRangeSD() {
        return mPseudoRangeSD;
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation) expressed in
     * meters (m).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final double pseudoRangeSD) {
        mPseudoRangeSD = pseudoRangeSD;
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation) expressed
     * in meters per second (m/s).
     *
     * @return pseudo-range rate measurement noise SD.
     */
    public double getRangeRateSD() {
        return mRangeRateSD;
    }

    /**
     * Sets pseudo-range rate measurement noise SD (Standard Deviation) expressed
     * in meters per second (m/s).
     *
     * @param rangeRateSD pseudo-range rate measurement noise SD.
     */
    public void setRangeRateSD(final double rangeRateSD) {
        mRangeRateSD = rangeRateSD;
    }

    /**
     * Sets configuration parameters.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public void setValues(final double gyroNoisePSD,
                          final double accelerometerNoisePSD,
                          final double accelerometerBiasPSD,
                          final double gyroBiasPSD,
                          final double clockFrequencyPSD,
                          final double clockPhasePSD,
                          final double pseudoRangeSD,
                          final double rangeRateSD) {
        mGyroNoisePSD = gyroNoisePSD;
        mAccelerometerNoisePSD = accelerometerNoisePSD;
        mAccelerometerBiasPSD = accelerometerBiasPSD;
        mGyroBiasPSD = gyroBiasPSD;
        mClockFrequencyPSD = clockFrequencyPSD;
        mClockPhasePSD = clockPhasePSD;
        mPseudoRangeSD = pseudoRangeSD;
        mRangeRateSD = rangeRateSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range measurement noise SD will be stored.
     */
    public void getPseudoRangeSDDistance(final Distance result) {
        result.setValue(mPseudoRangeSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range measurement noise SD.
     */
    public Distance getPseudoRangeSDDistance() {
        return new Distance(mPseudoRangeSD, DistanceUnit.METER);
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final Distance pseudoRangeSD) {
        mPseudoRangeSD = DistanceConverter.convert(
                pseudoRangeSD.getValue().doubleValue(),
                pseudoRangeSD.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range rate measurement noise SD will be
     *               stored.
     */
    public void getRangeRateSDSpeed(final Speed result) {
        result.setValue(mRangeRateSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range rate measurement noise SD.
     */
    public Speed getRangeRateSDSpeed() {
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
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public void setValues(final double gyroNoisePSD,
                          final double accelerometerNoisePSD,
                          final double accelerometerBiasPSD,
                          final double gyroBiasPSD,
                          final double clockFrequencyPSD,
                          final double clockPhasePSD,
                          final Distance pseudoRangeSD,
                          final Speed rangeRateSD) {
        mGyroNoisePSD = gyroNoisePSD;
        mAccelerometerNoisePSD = accelerometerNoisePSD;
        mAccelerometerBiasPSD = accelerometerBiasPSD;
        mGyroBiasPSD = gyroBiasPSD;
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
    public void copyTo(final INSTightlyCoupledKalmanConfig output) {
        output.mGyroNoisePSD = mGyroNoisePSD;
        output.mAccelerometerNoisePSD = mAccelerometerNoisePSD;
        output.mAccelerometerBiasPSD = mAccelerometerBiasPSD;
        output.mGyroBiasPSD = mGyroBiasPSD;
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
    public void copyFrom(final INSTightlyCoupledKalmanConfig input) {
        mGyroNoisePSD = input.mGyroNoisePSD;
        mAccelerometerNoisePSD = input.mAccelerometerNoisePSD;
        mAccelerometerBiasPSD = input.mAccelerometerBiasPSD;
        mGyroBiasPSD = input.mGyroBiasPSD;
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
        return Objects.hash(mGyroNoisePSD, mAccelerometerNoisePSD,
                mAccelerometerBiasPSD, mGyroBiasPSD, mClockFrequencyPSD,
                mClockPhasePSD, mPseudoRangeSD, mRangeRateSD);
    }

    /**
     * Checks if provided object is a INSTightlyCoupledKalmanConfig having exactly
     * the same contents as this instance.
     *
     * @param obj Object to be compared.
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
        final INSTightlyCoupledKalmanConfig other = (INSTightlyCoupledKalmanConfig) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final INSTightlyCoupledKalmanConfig other) {
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
    public boolean equals(final INSTightlyCoupledKalmanConfig other,
                          final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mGyroNoisePSD - other.mGyroNoisePSD) <= threshold
                && Math.abs(mAccelerometerNoisePSD - other.mAccelerometerNoisePSD) <= threshold
                && Math.abs(mAccelerometerBiasPSD - other.mAccelerometerBiasPSD) <= threshold
                && Math.abs(mGyroBiasPSD - other.mGyroBiasPSD) <= threshold
                && Math.abs(mClockFrequencyPSD - other.mClockFrequencyPSD) <= threshold
                && Math.abs(mClockPhasePSD - other.mClockPhasePSD) <= threshold
                && Math.abs(mPseudoRangeSD - other.mPseudoRangeSD) <= threshold
                && Math.abs(mRangeRateSD - other.mRangeRateSD) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final INSTightlyCoupledKalmanConfig result =
                (INSTightlyCoupledKalmanConfig) super.clone();
        copyTo(result);
        return result;
    }
}
