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
 * Contains magnetic flux density resolved around NED frame.
 */
public class NEDMagneticFluxDensity implements Serializable, Cloneable {

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * North component of magnetic flux density expressed in Teslas (T).
     */
    private double mBn;

    /**
     * East component of magnetic flux density expressed in Teslas (T).
     */
    private double mBe;

    /**
     * Down component of magnetic flux density expressed in Teslas (T).
     */
    private double mBd;

    /**
     * Constructor.
     */
    public NEDMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param bn north component of magnetic flux density expressed in
     *           Teslas (T).
     * @param be east component of magnetic flux density expressed in
     *           Teslas (T).
     * @param bd down component of magnetic flux density expressed in
     *           Teslas (T).
     */
    public NEDMagneticFluxDensity(
            final double bn, final double be, final double bd) {
        setCoordinates(bn, be, bd);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public NEDMagneticFluxDensity(final NEDMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets north component of magnetic flux density expressed in Teslas
     * (T).
     *
     * @return north component of magnetic flux density.
     */
    public double getBn() {
        return mBn;
    }

    /**
     * Sets north component of magnetic flux density expressed in Teslas
     * (T).
     *
     * @param bn north component of magnetic flux density.
     */
    public void setBn(final double bn) {
        mBn = bn;
    }

    /**
     * Gets east component of magnetic flux density expressed in Teslas
     * (T).
     *
     * @return east component of magnetic flux density.
     */
    public double getBe() {
        return mBe;
    }

    /**
     * Sets east component of magnetic flux density expressed in Teslas
     * (T).
     *
     * @param be est component of magnetic flux density.
     */
    public void setBe(final double be) {
        mBe = be;
    }

    /**
     * Gets down component of magnetic flux density expressed in Teslas
     * (T).
     *
     * @return down component of magnetic flux density.
     */
    public double getBd() {
        return mBd;
    }

    /**
     * Sets down component of magnetic flux density expressed in Teslas
     * (T).
     *
     * @param bd down component of magnetic flux density.
     */
    public void setBd(final double bd) {
        mBd = bd;
    }

    /**
     * Sets NED coordinates of magnetic flux density expressed in Teslas (T).
     *
     * @param bn north component of magnetic flux density.
     * @param be east component of magnetic flux density.
     * @param bd down component of magnetic flux density.
     */
    public void setCoordinates(
            final double bn, final double be, final double bd) {
        mBn = bn;
        mBe = be;
        mBd = bd;
    }

    /**
     * Gets magnetic flux density magnitude (e.g. norm) expressed in
     * Teslas (T).
     *
     * @return magnetic flux density magnitude.
     */
    public double getNorm() {
        return Math.sqrt(mBn * mBn + mBe * mBe + mBd * mBd);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDMagneticFluxDensity output) {
        output.mBn = mBn;
        output.mBe = mBe;
        output.mBd = mBd;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDMagneticFluxDensity input) {
        mBn = input.mBn;
        mBe = input.mBe;
        mBd = input.mBd;
    }

    /**
     * Gets magnetic flux density as an array.
     *
     * @param result array instance where magnetic flux density coordinates
     *               will be stored in n,e,d order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mBn;
        result[1] = mBe;
        result[2] = mBd;
    }

    /**
     * Gets magnetic flux density as an array.
     *
     * @return array containing magnetic flux density coordinates in n,e,d
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
     *               will be stored in n,e,d order.
     */
    public void asMatrix(final Matrix result) {
        if (result.getColumns() != COMPONENTS || result.getRows() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mBn);
        result.setElementAtIndex(1, mBe);
        result.setElementAtIndex(2, mBd);
    }

    /**
     * Gets magnetic flux density as a column matrix.
     *
     * @return a matrix containing magnetic flux density coordinates stored
     * in n,e,d order.
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
        return Objects.hash(mBn, mBe, mBd);
    }

    /**
     * Check if provided object is a NEDMagneticFluxDensity instance having
     * exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false
     * otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof NEDMagneticFluxDensity)) {
            return false;
        }

        final NEDMagneticFluxDensity other = (NEDMagneticFluxDensity) obj;
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
    public boolean equals(final NEDMagneticFluxDensity other) {
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
            final NEDMagneticFluxDensity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mBn - other.mBn) <= threshold
                && Math.abs(mBe - other.mBe) <= threshold
                && Math.abs(mBd - other.mBd) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for same reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final NEDMagneticFluxDensity result = (NEDMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }
}
