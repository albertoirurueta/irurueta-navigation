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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains acceleration due to gravity resolved about ECEF or ECI frame, depending
 * on the implementation.
 */
@SuppressWarnings("WeakerAccess")
public abstract class GravityOrGravitation<T extends GravityOrGravitation> implements Serializable, Cloneable {
    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * Acceleration due to gravity through ECI or ECEF x-axis expressed in meters per squared second (m/s^2).
     */
    double mGx;

    /**
     * Acceleration due to gravity through ECI or ECEF y-axis expressed in meters per squared second (m/s^2).
     */
    double mGy;

    /**
     * Acceleration due to gravity through ECI or ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    double mGz;

    /**
     * Constructor.
     */
    public GravityOrGravitation() {
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECI or ECEF x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECI or ECEF y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECI or ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    public GravityOrGravitation(final double gx, final double gy, final double gz) {
        setCoordinates(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param gx acceleration due to gravity through ECI or ECEF x-axis to be set.
     * @param gy acceleration due to gravity through ECI or ECEF y-axis to be set.
     * @param gz acceleration due to gravity through ECI or ECEF z-axis to be set.
     */
    public GravityOrGravitation(final Acceleration gx, final Acceleration gy,
                                final Acceleration gz) {
        setCoordinates(gx, gy, gz);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public GravityOrGravitation(final T input) {
        copyFrom(input);
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF x-axis expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through ECI or ECEF x-axis.
     */
    public double getGx() {
        return mGx;
    }

    /**
     * Sets acceleration due to gravity through ECI or ECEF x-axis expressed in meters per squared second (m/s^2).
     *
     * @param gx acceleration due to gravity through ECI or ECEF x-axis.
     */
    public void setGx(final double gx) {
        mGx = gx;
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF y-axis expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through ECI or ECEF y-axis.
     */
    public double getGy() {
        return mGy;
    }

    /**
     * Sets acceleration due to gravity through ECI or ECEF y-axis expressed in meters per squared second (m/s^2).
     *
     * @param gy acceleration due to gravity through ECI or ECEF y-axis.
     */
    public void setGy(final double gy) {
        mGy = gy;
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF z-axis expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through ECI or ECEF z-axis.
     */
    public double getGz() {
        return mGz;
    }

    /**
     * Sets acceleration due to gravity through ECI or ECEF z-axis expressed in meters per squared second (m/s^2).
     *
     * @param gz acceleration due to gravity through ECI or ECEF z-axis.
     */
    public void setGz(final double gz) {
        mGz = gz;
    }

    /**
     * Sets gravity coordinates resolved about ECI or ECEF frame and expressed in meters per squared second (m/s^2).
     *
     * @param gx acceleration due to gravity through ECI or ECEF x-axis expressed in meters per squared second (m/s^2).
     * @param gy acceleration due to gravity through ECI or ECEF y-axis expressed in meters per squared second (m/s^2).
     * @param gz acceleration due to gravity through ECI or ECEF z-axis expressed in meters per squared second (m/s^2).
     */
    public void setCoordinates(final double gx, final double gy, final double gz) {
        mGx = gx;
        mGy = gy;
        mGz = gz;
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF x-axis.
     *
     * @param result instance where acceleration due to gravity through ECI or ECEF x-axis will be stored.
     */
    public void getGxAsAcceleration(final Acceleration result) {
        result.setValue(mGx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF x-axis.
     *
     * @return acceleration due to gravity through ECI or ECEF x-axis.
     */
    public Acceleration getGxAsAcceleration() {
        return new Acceleration(mGx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through ECI or ECEF x-axis.
     *
     * @param gravityX acceleration due to gravity through ECI or ECEF x-axis to be set.
     */
    public void setGx(final Acceleration gravityX) {
        mGx = AccelerationConverter.convert(gravityX.getValue().doubleValue(),
                gravityX.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF y-axis.
     *
     * @param result instance where acceleration due to gravity through ECI or ECEF y-axis will be stored.
     */
    public void getGyAsAcceleration(final Acceleration result) {
        result.setValue(mGy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF y-axis.
     *
     * @return acceleration due to gravity through ECI or ECEF y-axis.
     */
    public Acceleration getGyAsAcceleration() {
        return new Acceleration(mGy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through ECI or ECEF y-axis.
     *
     * @param gravityY acceleration due to gravity through ECI or ECEF y-axis to be set.
     */
    public void setGy(final Acceleration gravityY) {
        mGy = AccelerationConverter.convert(gravityY.getValue().doubleValue(),
                gravityY.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF z-axis.
     *
     * @param result instance where acceleration due to gravity through ECI or ECEF z-axis will be stored.
     */
    public void getGzAsAcceleration(final Acceleration result) {
        result.setValue(mGz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through ECI or ECEF z-axis.
     *
     * @return acceleration due to gravity through ECI or ECEF z-axis.
     */
    public Acceleration getGzAsAcceleration() {
        return new Acceleration(mGz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through ECI or ECEF z-axis.
     *
     * @param gravityZ acceleration due to gravity through ECI or ECEF z-axis to be set.
     */
    public void setGz(final Acceleration gravityZ) {
        mGz = AccelerationConverter.convert(gravityZ.getValue().doubleValue(),
                gravityZ.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets gravity coordinates.
     *
     * @param gravityX acceleration due to gravity through ECI or ECEF x-axis to be set.
     * @param gravityY acceleration due to gravity through ECI or ECEF y-axis to be set.
     * @param gravityZ acceleration due to gravity through ECI or ECEF z-axis to be set.
     */
    public void setCoordinates(final Acceleration gravityX,
                               final Acceleration gravityY,
                               final Acceleration gravityZ) {
        setGx(gravityX);
        setGy(gravityY);
        setGz(gravityZ);
    }

    /**
     * Gets gravity norm.
     *
     * @return gravity norm.
     */
    public double getNorm() {
        return Math.sqrt(mGx * mGx + mGy * mGy + mGz * mGz);
    }

    /**
     * Gets gravity norm as an acceleration.
     *
     * @param result instance where result will be stored.
     */
    public void getNormAsAcceleration(final Acceleration result) {
        result.setValue(getNorm());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets gravity norm as an acceleration.
     *
     * @return an acceleration containing gravity norm.
     */
    public Acceleration getNormAsAcceleration() {
        return new Acceleration(getNorm(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final T output) {
        output.mGx = mGx;
        output.mGy = mGy;
        output.mGz = mGz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final T input) {
        mGx = input.mGx;
        mGy = input.mGy;
        mGz = input.mGz;
    }

    /**
     * Gets gravity coordinates expressed in meters per squared second (m/s^2) as an array.
     *
     * @param result array instance where gravity coordinates will be stored in
     *               x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mGx;
        result[1] = mGy;
        result[2] = mGz;
    }

    /**
     * Gets gravity coordinates expressed in meters per squared second (m/s^2) as an array.
     *
     * @return array containing gravity coordinates in x,y,z order.
     */
    public double[] asArray() {
        final double[] result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets gravity coordinates expressed in meters per squared second (m/s^2) as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where gravity coordinates will be stored in
     *               x,y,z order.
     */
    public void asMatrix(final Matrix result) {
        if (result.getRows() != COMPONENTS || result.getColumns() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mGx);
        result.setElementAtIndex(1, mGy);
        result.setElementAtIndex(2, mGz);
    }

    /**
     * Gets gravity coordinates as a column matrix.
     *
     * @return a matrix containing gravity coordinates stored in x,y,z order.
     */
    public Matrix asMatrix() {
        Matrix result;
        try {
            result = new Matrix(COMPONENTS, 1);
            asMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mGx, mGy, mGz);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final T other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between gravity coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final T other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mGx - other.mGx) <= threshold
                && Math.abs(mGy - other.mGy) <= threshold
                && Math.abs(mGz - other.mGz) <= threshold;
    }
}
