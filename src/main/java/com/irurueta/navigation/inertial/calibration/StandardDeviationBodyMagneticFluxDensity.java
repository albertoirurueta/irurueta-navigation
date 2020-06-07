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

import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a body magnetic flux density along with its
 * corresponding standard deviation.
 */
public class StandardDeviationBodyMagneticFluxDensity implements Serializable, Cloneable {

    /**
     * Current body magnetic flux density. Contains magnetometer measurements.
     */
    private BodyMagneticFluxDensity mMagneticFluxDensity;

    /**
     * Standard deviation of measured magnetic flux density expressed in Teslas
     * (T).
     */
    private double mMagneticFluxDensityStandardDeviation;

    /**
     * Constructor.
     */
    public StandardDeviationBodyMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     */
    public StandardDeviationBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensityStandardDeviation standard deviation of measured
     *                                             magnetic flux density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationBodyMagneticFluxDensity(
            final double magneticFluxDensityStandardDeviation) {
        setMagneticFluxDensityStandardDeviation(
                magneticFluxDensityStandardDeviation);
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity                  current body magnetic flux
     *                                             density.
     * @param magneticFluxDensityStandardDeviation standard deviation of measured
     *                                             magnetic flux density.
     * @throws IllegalArgumentException if provided standard deviation is
     *                                  negative.
     */
    public StandardDeviationBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity,
            final double magneticFluxDensityStandardDeviation) {
        this(magneticFluxDensityStandardDeviation);
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public StandardDeviationBodyMagneticFluxDensity(
            final StandardDeviationBodyMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets current body magnetic flux density. Contains magnetometer
     * measurements.
     *
     * @return current body magnetic flux density.
     */
    public BodyMagneticFluxDensity getMagneticFluxDensity() {
        return mMagneticFluxDensity;
    }

    /**
     * Sets current body magnetic flux density. Contains magnetometer
     * measurements.
     *
     * @param magneticFluxDensity current body magnetic flux density.
     */
    public void setMagneticFluxDensity(
            final BodyMagneticFluxDensity magneticFluxDensity) {
        mMagneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Gets standard deviation of measured magnetic flux density expressed in
     * Teslas (T).
     *
     * @return standard deviation of measured magnetic flux density.
     */
    public double getMagneticFluxDensityStandardDeviation() {
        return mMagneticFluxDensityStandardDeviation;
    }

    /**
     * Sets standard deviation of measured magnetic flux density expressed in
     * Teslas (T).
     *
     * @param magneticFluxDensityStandardDeviation standard deviation of
     *                                             measured magnetic flux
     *                                             density.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setMagneticFluxDensityStandardDeviation(
            final double magneticFluxDensityStandardDeviation) {
        if (magneticFluxDensityStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mMagneticFluxDensityStandardDeviation =
                magneticFluxDensityStandardDeviation;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final StandardDeviationBodyMagneticFluxDensity input) {
        if (input.mMagneticFluxDensity != null) {
            if (mMagneticFluxDensity == null) {
                mMagneticFluxDensity = new BodyMagneticFluxDensity(
                        input.mMagneticFluxDensity);
            } else {
                mMagneticFluxDensity.copyFrom(input.mMagneticFluxDensity);
            }
        } else {
            mMagneticFluxDensity = null;
        }

        mMagneticFluxDensityStandardDeviation =
                input.mMagneticFluxDensityStandardDeviation;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final StandardDeviationBodyMagneticFluxDensity output) {
        output.copyFrom(this);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost
     * unique values that are useful for fast classification and storage of
     * objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mMagneticFluxDensity,
                mMagneticFluxDensityStandardDeviation);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(
            final StandardDeviationBodyMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to
     * provided threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between magnetic flux density
     *                  and standard deviation value.
     * @return true if both instances are considered to be equal (up to provided
     * threshold, false otherwise).
     */
    public boolean equals(
            final StandardDeviationBodyMagneticFluxDensity other,
            final double threshold) {
        if (other == null) {
            return false;
        }

        return ((other.mMagneticFluxDensity == null && mMagneticFluxDensity == null)
                || (mMagneticFluxDensity != null && mMagneticFluxDensity.equals(other.mMagneticFluxDensity, threshold)))
                && Math.abs(mMagneticFluxDensityStandardDeviation - other.mMagneticFluxDensityStandardDeviation) <= threshold;
    }

    /**
     * Checks if provided object is a StandardDeviationBodyMagneticFluxDensity
     * instance having exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        final StandardDeviationBodyMagneticFluxDensity other =
                (StandardDeviationBodyMagneticFluxDensity) obj;
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
        final StandardDeviationBodyMagneticFluxDensity result =
                (StandardDeviationBodyMagneticFluxDensity) super.clone();
        copyTo(result);
        return super.clone();
    }
}
