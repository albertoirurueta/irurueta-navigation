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
public class INSLooselyCoupledKalmanConfig implements Serializable, Cloneable {

    /**
     * Gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     */
    private double mGyroNoisePSD;

    /**
     * Accelerometer noise PSD (Power Spectral Density) expressed in (m^2 * s^-3).
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
     * Position measurement noise SD (Standard Deviation) per axis expressed in
     * meters (m).
     */
    private double mPositionNoiseSD;

    /**
     * Velocity measurement noise SD (Standard Deviation) per axis expressed in
     * meters per second (m/s).
     */
    private double mVelocityNoiseSD;

    /**
     * Constructor.
     */
    public INSLooselyCoupledKalmanConfig() {
    }

    /**
     * Constructor.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed in
     *                              squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param positionNoiseSD       position measurement noise SD (Standard Deviation)
     *                              per axis expressed in meters (m).
     * @param velocityNoiseSD       velocity measurement noise SD (Standard Deviation)
     *                              per axis expressed in meters per second (m/s).
     */
    public INSLooselyCoupledKalmanConfig(final double gyroNoisePSD,
                                         final double accelerometerNoisePSD,
                                         final double accelerometerBiasPSD,
                                         final double gyroBiasPSD,
                                         final double positionNoiseSD,
                                         final double velocityNoiseSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
    }

    /**
     * Constructor.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed in
     *                              squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param positionNoiseSD       position measurement noise SD (Standard Deviation)
     *                              per axis.
     * @param velocityNoiseSD       velocity measurement noise SD (Standard Deviation)
     *                              per axis.
     */
    public INSLooselyCoupledKalmanConfig(final double gyroNoisePSD,
                                         final double accelerometerNoisePSD,
                                         final double accelerometerBiasPSD,
                                         final double gyroBiasPSD,
                                         final Distance positionNoiseSD,
                                         final Speed velocityNoiseSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
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
     * Sets accelerometer noise PSD (Power Spectral Density) expressed in (m^2 * s^-3).
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
     * Gets position measurement noise SD (Standard Deviation) per axis expressed
     * in meters (m).
     *
     * @return position measurement noise SD.
     */
    public double getPositionNoiseSD() {
        return mPositionNoiseSD;
    }

    /**
     * Sets position measurement noise SD (Standard Deviation) per axis expressed
     * in meters (m).
     *
     * @param positionNoiseSD position measurement noise SD.
     */
    public void setPositionNoiseSD(final double positionNoiseSD) {
        mPositionNoiseSD = positionNoiseSD;
    }

    /**
     * Gets velocity measurement noise SD (Standard Deviation) per axis expressed in
     * meters per second (m/s).
     *
     * @return velocity measurement noise SD.
     */
    public double getVelocityNoiseSD() {
        return mVelocityNoiseSD;
    }

    /**
     * Sets velocity measurement noise SD (Standard Deviation) per axis expressed in
     * meters per second (m/s).
     *
     * @param velocityNoiseSD velocity measurement noise SD.
     */
    public void setVelocityNoiseSD(final double velocityNoiseSD) {
        mVelocityNoiseSD = velocityNoiseSD;
    }

    /**
     * Sets configuration parameters.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed in
     *                              squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param positionNoiseSD       position measurement noise SD (Standard Deviation)
     *                              per axis expressed in meters (m).
     * @param velocityNoiseSD       velocity measurement noise SD (Standard Deviation)
     *                              per axis expressed in meters per second (m/s).
     */
    public void setValues(final double gyroNoisePSD,
                          final double accelerometerNoisePSD,
                          final double accelerometerBiasPSD,
                          final double gyroBiasPSD,
                          final double positionNoiseSD,
                          final double velocityNoiseSD) {
        mGyroNoisePSD = gyroNoisePSD;
        mAccelerometerNoisePSD = accelerometerNoisePSD;
        mAccelerometerBiasPSD = accelerometerBiasPSD;
        mGyroBiasPSD = gyroBiasPSD;
        mPositionNoiseSD = positionNoiseSD;
        mVelocityNoiseSD = velocityNoiseSD;
    }

    /**
     * Gets position measurement noise SD (Standard Deviation) per axis.
     *
     * @param result instance where position measurement noise SD will be stored.
     */
    public void getPositionNoiseSDAsDistance(final Distance result) {
        result.setValue(mPositionNoiseSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets position measurement noise SD (Standard Deviation) per axis.
     *
     * @return position measurement noise SD.
     */
    public Distance getPositionNoiseSDAsDistance() {
        return new Distance(mPositionNoiseSD, DistanceUnit.METER);
    }

    /**
     * Sets position measurement noise SD (Standard Deviation) per axis.
     *
     * @param positionNoiseSD position measurement noise SD.
     */
    public void setPositionNoiseSD(final Distance positionNoiseSD) {
        mPositionNoiseSD = DistanceConverter.convert(
                positionNoiseSD.getValue().doubleValue(),
                positionNoiseSD.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets velocity measurement noise SD (Standard Deviation) per axis.
     *
     * @param result instance where velocity measurement noise SD will be stored.
     */
    public void getVelocityNoiseSDAsSpeed(final Speed result) {
        result.setValue(mVelocityNoiseSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets velocity measurement noise SD (Standard Deviation) per axis.
     *
     * @return velocity measurement noise SD per axis.
     */
    public Speed getVelocityNoiseSDAsSpeed() {
        return new Speed(mVelocityNoiseSD, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets velocity measurement noise SD (Standard Deviation) per axis.
     *
     * @param velocityNoiseSD velocity measurement noise SD per axis.
     */
    public void setVelocityNoiseSD(final Speed velocityNoiseSD) {
        mVelocityNoiseSD = SpeedConverter.convert(
                velocityNoiseSD.getValue().doubleValue(),
                velocityNoiseSD.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets configuration parameters.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed in
     *                              squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param positionNoiseSD       position measurement noise SD (Standard Deviation)
     *                              per axis.
     * @param velocityNoiseSD       velocity measurement noise SD (Standard Deviation)
     *                              per axis.
     */
    public void setValues(final double gyroNoisePSD,
                          final double accelerometerNoisePSD,
                          final double accelerometerBiasPSD,
                          final double gyroBiasPSD,
                          final Distance positionNoiseSD,
                          final Speed velocityNoiseSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD,
                DistanceConverter.convert(positionNoiseSD.getValue().doubleValue(),
                        positionNoiseSD.getUnit(), DistanceUnit.METER),
                SpeedConverter.convert(velocityNoiseSD.getValue().doubleValue(),
                        velocityNoiseSD.getUnit(), SpeedUnit.METERS_PER_SECOND));
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final INSLooselyCoupledKalmanConfig output) {
        output.mGyroNoisePSD = mGyroNoisePSD;
        output.mAccelerometerNoisePSD = mAccelerometerNoisePSD;
        output.mAccelerometerBiasPSD = mAccelerometerBiasPSD;
        output.mGyroBiasPSD = mGyroBiasPSD;
        output.mPositionNoiseSD = mPositionNoiseSD;
        output.mVelocityNoiseSD = mVelocityNoiseSD;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final INSLooselyCoupledKalmanConfig input) {
        mGyroNoisePSD = input.mGyroNoisePSD;
        mAccelerometerNoisePSD = input.mAccelerometerNoisePSD;
        mAccelerometerBiasPSD = input.mAccelerometerBiasPSD;
        mGyroBiasPSD = input.mGyroBiasPSD;
        mPositionNoiseSD = input.mPositionNoiseSD;
        mVelocityNoiseSD = input.mVelocityNoiseSD;
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
                mAccelerometerBiasPSD, mGyroBiasPSD, mPositionNoiseSD,
                mVelocityNoiseSD);
    }

    /**
     * Checks if provided object is a INSLooselyCoupledKalmanConfig having exactly
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
        final INSLooselyCoupledKalmanConfig other = (INSLooselyCoupledKalmanConfig) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final INSLooselyCoupledKalmanConfig other) {
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
    public boolean equals(final INSLooselyCoupledKalmanConfig other,
                          final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mGyroNoisePSD - other.mGyroNoisePSD) <= threshold
                && Math.abs(mAccelerometerNoisePSD - other.mAccelerometerNoisePSD) <= threshold
                && Math.abs(mAccelerometerBiasPSD - other.mAccelerometerBiasPSD) <= threshold
                && Math.abs(mGyroBiasPSD - other.mGyroBiasPSD) <= threshold
                && Math.abs(mPositionNoiseSD - other.mPositionNoiseSD) <= threshold
                && Math.abs(mVelocityNoiseSD - other.mVelocityNoiseSD) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final INSLooselyCoupledKalmanConfig result =
                (INSLooselyCoupledKalmanConfig) super.clone();
        copyTo(result);
        return result;
    }
}
