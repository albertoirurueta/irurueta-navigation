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
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

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
     * @param bn north component of magnetic flux density.
     * @param be east component of magnetic flux density.
     * @param bd down component of magnetic flux density.
     */
    public NEDMagneticFluxDensity(
            final MagneticFluxDensity bn,
            final MagneticFluxDensity be,
            final MagneticFluxDensity bd) {
        setCoordinates(bn, be, bd);
    }

    /**
     * Constructor.
     *
     * @param triad triad containing magnetic flux density values, where x,y,z
     *              coordinates correspond to north, east, down coordinates.
     */
    public NEDMagneticFluxDensity(final MagneticFluxDensityTriad triad) {
        setCoordinates(triad);
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
     * Gets north component of magnetic flux density.
     *
     * @return north component of magnetic flux density.
     */
    public MagneticFluxDensity getBnAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mBn, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets north component of magnetic flux density.
     *
     * @param result instance where result will be stored.
     */
    public void getBnAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mBn);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets north component of magnetic flux density.
     *
     * @param bn north component of magnetic flux density.
     */
    public void setBn(final MagneticFluxDensity bn) {
        mBn = convertMagneticFluxDensity(bn);
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
     * Gets east component of magnetic flux density.
     *
     * @return east component of magnetic flux density.
     */
    public MagneticFluxDensity getBeAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mBe, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets east component of magnetic flux density.
     *
     * @param result instance where result will be stored.
     */
    public void getBeAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mBe);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets east component of magnetic flux density.
     *
     * @param be east component of magnetic flux density.
     */
    public void setBe(final MagneticFluxDensity be) {
        mBe = convertMagneticFluxDensity(be);
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
     * Gets down component of magnetic flux density.
     *
     * @return down component of magnetic flux density.
     */
    public MagneticFluxDensity getBdAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mBd, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets down component of magnetic flux density.
     *
     * @param result instance where result will be stored.
     */
    public void getBdAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mBd);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets down component of magnetic flux density.
     *
     * @param bd down component of magnetic flux density.
     */
    public void setBd(final MagneticFluxDensity bd) {
        mBd = convertMagneticFluxDensity(bd);
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
     * Sets NED coordinates of magnetic flux density.
     *
     * @param bn north component of magnetic flux density.
     * @param be east component of magnetic flux density.
     * @param bd down component of magnetic flux density.
     */
    public void setCoordinates(
            final MagneticFluxDensity bn,
            final MagneticFluxDensity be,
            final MagneticFluxDensity bd) {
        setCoordinates(convertMagneticFluxDensity(bn),
                convertMagneticFluxDensity(be), convertMagneticFluxDensity(bd));
    }

    /**
     * Gets NED coordinates of magnetic flux density as a triad.
     * x,y,z coordinates correspond to north, east, down coordinates.
     *
     * @return NED coordinates of magnetic flux density as a triad.
     */
    public MagneticFluxDensityTriad getCoordinatesAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, mBn, mBe, mBd);
    }

    /**
     * Gets NED coordinates of magnetic flux density as a triad.
     * x,y,z coordinates correspond to north, east, down coordinates.
     *
     * @param result instance where result will be stored.
     */
    public void getCoordinatesAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(mBn, mBe, mBd, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets NED coordinates of magnetic flux density.
     *
     * @param triad triad containing magnetic flux density values, where x,y,z
     *              coordinates correspond to north, east, down coordinates.
     */
    public void setCoordinates(final MagneticFluxDensityTriad triad) {
        final double bn = convertMagneticFluxDensity(triad.getValueX(), triad.getUnit());
        final double be = convertMagneticFluxDensity(triad.getValueY(), triad.getUnit());
        final double bd = convertMagneticFluxDensity(triad.getValueZ(), triad.getUnit());
        setCoordinates(bn, be, bd);
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
     * Gets magnetic flux density magnitude (e.g. norm).
     *
     * @return magnetic flux density magnitude.
     */
    public MagneticFluxDensity getNormAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getNorm(), MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets magnetic flux density magnitude (e.g. norm).
     *
     * @param result instance where result will be stored.
     */
    public void getNormAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(getNorm());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
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

    /**
     * Converts magnetic flux density to Teslas.
     *
     * @param b magnetic flux density to be converted.
     * @return converted value.
     */
    private double convertMagneticFluxDensity(final MagneticFluxDensity b) {
        return MagneticFluxDensityConverter.convert(b.getValue().doubleValue(),
                b.getUnit(), MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Converts magnetic flux density to Teslas.
     *
     * @param value value to be converted.
     * @param unit unit of value to be converted
     * @return converted value.
     */
    private double convertMagneticFluxDensity(
            final double value, final MagneticFluxDensityUnit unit) {
        return MagneticFluxDensityConverter.convert(value, unit, MagneticFluxDensityUnit.TESLA);
    }
}
