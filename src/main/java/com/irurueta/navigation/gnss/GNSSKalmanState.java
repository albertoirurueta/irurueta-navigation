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

import com.irurueta.algebra.Matrix;

import java.io.Serializable;
import java.util.Objects;

/**
 * Kalman filter state for filtered GNSS estimation.
 */
public class GNSSKalmanState implements Serializable, Cloneable {

    /**
     * Contains estimation of ECEF position and velocity, and estimated clock
     * offset and drift.
     */
    private GNSSEstimation mEstimation;

    /**
     * Kalman filter error covariance matrix.
     */
    private Matrix mCovariance;

    /**
     * Constructor.
     */
    public GNSSKalmanState() {
    }

    /**
     * Constructor.
     *
     * @param estimation GNSS estimation containing ECEF position, velocity and clock
     *                   offset and drift.
     * @param covariance Kalman filter error covariance matrix. Must be 8x8.
     * @throws IllegalArgumentException if provided covariance matrix is not 8x8.
     */
    public GNSSKalmanState(final GNSSEstimation estimation, final Matrix covariance) {
        setEstimation(estimation);
        setCovariance(covariance);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public GNSSKalmanState(final GNSSKalmanState input) {
        copyFrom(input);
    }

    /**
     * Gets estimation of ECEF position and velocity, and estimated clock offset
     * and drift.
     * If GNSS estimation data is not available, this method makes no action.
     *
     * @param result instance where result data will be copied to.
     * @return true if result data has been copied, false otherwise.
     */
    public boolean getEstimation(final GNSSEstimation result) {
        if (mEstimation != null) {
            mEstimation.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimation of ECEF position and velocity, and estimated clock offset
     * and drift.
     *
     * @return GNSS estimation.
     */
    public GNSSEstimation getEstimation() {
        return mEstimation;
    }

    /**
     * Sets estimation of ECEF position and velocity, and estimated clock offset
     * and drift.
     *
     * @param estimation GNSS estimation to be set.
     */
    public void setEstimation(final GNSSEstimation estimation) {
        mEstimation = estimation;
    }

    /**
     * Gets Kalman filter error covariance matrix.
     *
     * @param result instance where result data will be copied to.
     * @return true if result data has been copied, false otherwise.
     */
    public boolean getCovariance(final Matrix result) {
        if (mCovariance != null) {
            mCovariance.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets Kalman filter error covariance matrix.
     *
     * @return Kalman filter error covariance matrix.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Sets Kalman filter error covariance matrix.
     *
     * @param covariance Kalman filter error covariance matrix to be set.
     * @throws IllegalArgumentException if provided covariance matrix is not 8x8.
     */
    public void setCovariance(final Matrix covariance) {
        if (covariance.getRows() != GNSSEstimation.NUM_PARAMETERS ||
                covariance.getColumns() != GNSSEstimation.NUM_PARAMETERS) {
            throw new IllegalArgumentException();
        }

        mCovariance = covariance;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final GNSSKalmanState output) {
        output.copyFrom(this);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final GNSSKalmanState input) {
        // copy estimation
        if (input.mEstimation == null) {
            mEstimation = null;
        } else {
            if (mEstimation == null) {
                mEstimation = new GNSSEstimation();
            }
            mEstimation.copyFrom(input.mEstimation);
        }

        // copy covariance
        if (input.mCovariance == null) {
            mCovariance = null;
        } else {
            if (mCovariance == null) {
                mCovariance = new Matrix(input.mCovariance);
            } else {
                mCovariance.copyFrom(input.mCovariance);
            }
        }
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mEstimation, mCovariance);
    }

    /**
     * Checks if provided object is a GNSSKalmanState having exactly the same
     * contents as this instance.
     *
     * @param obj Object to be compared.
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
        final GNSSKalmanState other = (GNSSKalmanState) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final GNSSKalmanState other) {
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
    public boolean equals(final GNSSKalmanState other, final double threshold) {
        if (other == null) {
            return false;
        }

        return other.mEstimation != null
                && other.mEstimation.equals(mEstimation, threshold)
                && other.mCovariance != null
                && other.mCovariance.equals(mCovariance, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final GNSSKalmanState result = (GNSSKalmanState) super.clone();
        copyTo(result);
        return result;
    }
}
