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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains magnetic flux density resolved around body coordinates.
 * Body frame axes are typically defined so that x is the forward axis, pointing in the usual direction
 * of travel, z is the down axis, pointing in the usual direction of gravity, and y is the right axis,
 * completing the orthogonal set.
 */
public class BodyMagneticFluxDensity implements Serializable, Cloneable {

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * X component of magnetic flux density expressed in Teslas (T).
     */
    private double mBx;

    /**
     * Y component of magnetic flux density expressed in Teslas (T).
     */
    private double mBy;

    /**
     * Z component of magnetic flux density expressed in Teslas (T).
     */
    private double mBz;

    /**
     * Constructor.
     */
    public BodyMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param bx x component of magnetic flux density expressed in Teslas (T).
     * @param by y component of magnetic flux density expressed in Teslas (T).
     * @param bz z component of magnetic flux density expressed in Teslas (T).
     */
    public BodyMagneticFluxDensity(
            final double bx, final double by, final double bz) {
        setCoordinates(bx, by, bz);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyMagneticFluxDensity(final BodyMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets x component of magnetic flux density expressed in Teslas (T).
     *
     * @return x component of magnetic flux density.
     */
    public double getBx() {
        return mBx;
    }

    /**
     * Sets x component of magnetic flux density expressed in Teslas (T).
     *
     * @param bx x component of magnetic flux density.
     */
    public void setBx(final double bx) {
        mBx = bx;
    }

    /**
     * Gets y component of magnetic flux density expressed in Teslas (T).
     *
     * @return y component of magnetic flux density.
     */
    public double getBy() {
        return mBy;
    }

    /**
     * Sets y component of magnetic flux density expresssed in Teslas (T).
     *
     * @param by y component of magnetic flux density.
     */
    public void setBy(final double by) {
        mBy = by;
    }

    /**
     * Gets z component of magnetic flux density expressed in Teslas (T).
     *
     * @return z component of magnetic flux density.
     */
    public double getBz() {
        return mBz;
    }

    /**
     * Sets z component of magnetic flux density expressed in Teslas (T).
     *
     * @param bz z component of magnetic flux density.
     */
    public void setBz(final double bz) {
        mBz = bz;
    }

    /**
     * Sets body coordinates of magnetic flux density expressed in Teslas (T).
     *
     * @param bx x component of magnetic flux density.
     * @param by y component of magnetic flux density.
     * @param bz z component of magnetic flux density.
     */
    public void setCoordinates(
            final double bx, final double by, final double bz) {
        mBx = bx;
        mBy = by;
        mBz = bz;
    }

    /**
     * Gets magnetic flux density magnitude (e.g. norm) expressed in
     * Teslas (T).
     *
     * @return magnetic flux density magnitude.
     */
    public double getNorm() {
        return Math.sqrt(mBx * mBx + mBy * mBy + mBz * mBz);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyMagneticFluxDensity output) {
        output.mBx = mBx;
        output.mBy = mBy;
        output.mBz = mBz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final BodyMagneticFluxDensity input) {
        mBx = input.mBx;
        mBy = input.mBy;
        mBz = input.mBz;
    }

    /**
     * Gets magnetic flux density as an array.
     *
     * @param result array instance where magnetic flux density coordinates
     *               will be stored in x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mBx;
        result[1] = mBy;
        result[2] = mBz;
    }

    /**
     * Gets magnetic flux density as an array.
     *
     * @return array containing magnetic flux density coordinates in x,y,z
     * order.
     */
    public double[] asArray() {
        final double[] result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets magnetic flux density as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where magnetic flux density coordinates
     *               will be stored in x,y,z order.
     */
    public void asMatrix(final Matrix result) {
        if (result.getColumns() != COMPONENTS || result.getRows() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mBx);
        result.setElementAtIndex(1, mBy);
        result.setElementAtIndex(2, mBz);
    }

    /**
     * Gets magnetic flux density as a column matrix.
     *
     * @return a matrix containing magnetic flux density coordinates stored
     * in x,y,z order.
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
        return Objects.hash(mBx, mBy, mBz);
    }

    /**
     * Check if provided object is a BodyMagneticFluxDensity instance having
     * exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false
     * otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof BodyMagneticFluxDensity)) {
            return false;
        }

        final BodyMagneticFluxDensity other = (BodyMagneticFluxDensity) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this
     * instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false
     * otherwise.
     */
    public boolean equals(final BodyMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up
     * to provided threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between gravity coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(
            final BodyMagneticFluxDensity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mBx - other.mBx) <= threshold
                && Math.abs(mBy - other.mBy) <= threshold
                && Math.abs(mBz - other.mBz) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for same reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final BodyMagneticFluxDensity result = (BodyMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }
}
