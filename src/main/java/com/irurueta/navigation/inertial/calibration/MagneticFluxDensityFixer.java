/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

/**
 * Fixes magnetic flux density values taking into account provided bias and
 * cross coupling errors.
 */
public class MagneticFluxDensityFixer {
    /**
     * Identity matrix to be reused.
     */
    private Matrix mIdentity;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mTmp1;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mTmp2;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mDiff;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mTmp3;

    /**
     * Measured specific force array to be reused.
     */
    private final double[] mMeasuredB =
            new double[MagneticFluxDensityTriad.COMPONENTS];

    /**
     * Array containing result values to be reused.
     */
    private final double[] mResult =
            new double[MagneticFluxDensityTriad.COMPONENTS];

    /**
     * Bias matrix to be reused.
     */
    private Matrix mBias;

    /**
     * Cross coupling errors matrix to be reused.
     */
    private Matrix mCrossCouplingErrors;

    /**
     * Constructor.
     */
    public MagneticFluxDensityFixer() {
        try {
            mIdentity = Matrix.identity(MagneticFluxDensityTriad.COMPONENTS,
                    MagneticFluxDensityTriad.COMPONENTS);
            mTmp1 = Matrix.identity(MagneticFluxDensityTriad.COMPONENTS,
                    MagneticFluxDensityTriad.COMPONENTS);
            mTmp2 = Matrix.identity(MagneticFluxDensityTriad.COMPONENTS,
                    MagneticFluxDensityTriad.COMPONENTS);
            mDiff = new Matrix(MagneticFluxDensityTriad.COMPONENTS, 1);
            mTmp3 = new Matrix(MagneticFluxDensityTriad.COMPONENTS, 1);

            mBias = new Matrix(MagneticFluxDensityTriad.COMPONENTS, 1);
            mCrossCouplingErrors = new Matrix(MagneticFluxDensityTriad.COMPONENTS,
                    MagneticFluxDensityTriad.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets bias values expressed in Teslas (T).
     *
     * @return bias values expressed in Teslas.
     */
    public Matrix getBias() {
        return new Matrix(mBias);
    }

    /**
     * Gets bias values expressed in Teslas (T).
     *
     * @param result instance where result will be stored.
     */
    public void getBias(final Matrix result) {
        mBias.copyTo(result);
    }

    /**
     * Sets bias values expressed in Teslas (T).
     *
     * @param bias bias values expressed in Teslas. Must be 3x1.
     */
    public void setBias(final Matrix bias) {
        if (bias.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        mBias = bias;
    }

    /**
     * Gets bias values expressed in Teslas (T).
     *
     * @return bias values expressed in Teslas.
     */
    public double[] getBiasArray() {
        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        getBiasArray(result);
        return result;
    }

    /**
     * Gets bias values expressed in Teslas (T).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void getBiasArray(final double[] result) {
        if (result.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        try {
            mBias.toArray(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Sets bias values expressed in Teslas (T).
     *
     * @param bias bias values expressed in Teslas (T). Must have length 3.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void setBias(final double[] bias) {
        if (bias.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        try {
            mBias.fromArray(bias);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets bias.
     *
     * @return bias.
     */
    public BodyMagneticFluxDensity getBiasAsBodyMagneticFluxDensity() {
        return new BodyMagneticFluxDensity(mBias.getElementAtIndex(0),
                mBias.getElementAtIndex(1), mBias.getElementAtIndex(2));
    }

    /**
     * Gets bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsBodyMagneticFluxDensity(
            final BodyMagneticFluxDensity result) {
        result.setCoordinates(mBias.getElementAtIndex(0),
                mBias.getElementAtIndex(1), mBias.getElementAtIndex(2));
    }

    /**
     * Sets bias.
     *
     * @param bias bias to be set.
     */
    public void setBias(final BodyMagneticFluxDensity bias) {
        mBias.setElementAtIndex(0, bias.getBx());
        mBias.setElementAtIndex(1, bias.getBy());
        mBias.setElementAtIndex(2, bias.getBz());
    }

    /**
     * Gets bias.
     *
     * @return bias.
     */
    public MagneticFluxDensityTriad getBiasAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                mBias.getElementAtIndex(0), mBias.getElementAtIndex(1),
                mBias.getElementAtIndex(2));
    }

    /**
     * Gets bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinates(mBias);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets bias.
     *
     * @param bias bias to be set.
     */
    public void setBias(final MagneticFluxDensityTriad bias) {
        final double biasX = convertMagneticFluxDensity(bias.getValueX(), bias.getUnit());
        final double biasY = convertMagneticFluxDensity(bias.getValueY(), bias.getUnit());
        final double biasZ = convertMagneticFluxDensity(bias.getValueZ(), bias.getUnit());
        mBias.setElementAtIndex(0, biasX);
        mBias.setElementAtIndex(1, biasY);
        mBias.setElementAtIndex(2, biasZ);
    }

    /**
     * Gets x-coordinate of bias expressed in Teslas (T).
     *
     * @return x-coordinate of bias expressed in Teslas (T).
     */
    public double getBiasX() {
        return mBias.getElementAtIndex(0);
    }

    /**
     * Sets x-coordinate of bias expressed in Teslas (T).
     *
     * @param biasX x-coordinate of bias expressed in Teslas (T).
     */
    public void setBiasX(final double biasX) {
        mBias.setElementAtIndex(0, biasX);
    }

    /**
     * Gets y-coordinate of bias expressed in Teslas (T).
     *
     * @return y-coordinate of bias expressed in Teslas (T).
     */
    public double getBiasY() {
        return mBias.getElementAtIndex(1);
    }

    /**
     * Sets y-coordinate of bias expressed in Teslas (T).
     *
     * @param biasY y-coordinate of bias expressed in Teslas (T).
     */
    public void setBiasY(final double biasY) {
        mBias.setElementAtIndex(1, biasY);
    }

    /**
     * Gets z-coordinate of bias expressed in Teslas (T).
     *
     * @return z-coordinate of bias expressed in Teslas (T).
     */
    public double getBiasZ() {
        return mBias.getElementAtIndex(2);
    }

    /**
     * Sets z-coordinate of bias expressed in Teslas (T).
     *
     * @param biasZ z-coordinate of bias expressed in Teslas (T).
     */
    public void setBiasZ(final double biasZ) {
        mBias.setElementAtIndex(2, biasZ);
    }

    /**
     * Sets coordinates of bias expressed in Teslas (T).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setBias(
            final double biasX, final double biasY, final double biasZ) {
        setBiasX(biasX);
        setBiasY(biasY);
        setBiasZ(biasZ);
    }

    /**
     * Gets x-coordinate of bias.
     *
     * @return x-coordinate of bias.
     */
    public MagneticFluxDensity getBiasXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getBiasX(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(getBiasX());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets x-coordinate of bias.
     *
     * @param biasX x-coordinate of bias.
     */
    public void setBiasX(final MagneticFluxDensity biasX) {
        setBiasX(convertMagneticFluxDensity(biasX));
    }

    /**
     * Gets y-coordinate of bias.
     *
     * @return y-coordinate of bias.
     */
    public MagneticFluxDensity getBiasYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getBiasY(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(getBiasY());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets y-coordinate of bias.
     *
     * @param biasY y-coordinate of bias.
     */
    public void setBiasY(final MagneticFluxDensity biasY) {
        setBiasY(convertMagneticFluxDensity(biasY));
    }

    /**
     * Gets z-coordinate of bias.
     *
     * @return z-coordinate of bias.
     */
    public MagneticFluxDensity getBiasZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getBiasZ(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(getBiasZ());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     */
    public void setBiasZ(final MagneticFluxDensity biasZ) {
        setBiasZ(convertMagneticFluxDensity(biasZ));
    }

    /**
     * Sets coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setBias(
            final MagneticFluxDensity biasX,
            final MagneticFluxDensity biasY,
            final MagneticFluxDensity biasZ) {
        setBiasX(biasX);
        setBiasY(biasY);
        setBiasZ(biasZ);
    }

    /**
     * Gets cross coupling errors matrix.
     *
     * @return cross coupling errors matrix.
     */
    public Matrix getCrossCouplingErrors() {
        return new Matrix(mCrossCouplingErrors);
    }

    /**
     * Gets cross coupling errors matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getCrossCouplingErrors(final Matrix result) {
        mCrossCouplingErrors.copyTo(result);
    }

    /**
     * Sets cross coupling errors matrix.
     *
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided matrix cannot be inverted.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setCrossCouplingErrors(final Matrix crossCouplingErrors)
            throws AlgebraException {
        if (crossCouplingErrors.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || crossCouplingErrors.getColumns() != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mCrossCouplingErrors = crossCouplingErrors;

        mIdentity.add(crossCouplingErrors, mTmp1);

        Utils.inverse(mTmp1, mTmp2);
    }

    /**
     * Gets x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getSx() {
        return mCrossCouplingErrors.getElementAt(0, 0);
    }

    /**
     * Sets x scaling factor
     *
     * @param sx x scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSx(final double sx) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(0, 0, sx);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getSy() {
        return mCrossCouplingErrors.getElementAt(1, 1);
    }

    /**
     * Sets y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSy(final double sy) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(1, 1, sy);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getSz() {
        return mCrossCouplingErrors.getElementAt(2, 2);
    }

    /**
     * Sets z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSz(final double sz) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(2, 2, sz);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets x-y cross coupling error.
     *
     * @return x-y cross coupling error.
     */
    public double getMxy() {
        return mCrossCouplingErrors.getElementAt(0, 1);
    }

    /**
     * Sets x-y cross coupling error.
     *
     * @param mxy x-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMxy(final double mxy) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(0, 1, mxy);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets x-z cross coupling error.
     *
     * @return x-z cross coupling error.
     */
    public double getMxz() {
        return mCrossCouplingErrors.getElementAt(0, 2);
    }

    /**
     * Sets x-z cross coupling error.
     *
     * @param mxz x-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMxz(final double mxz) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(0, 2, mxz);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets y-x cross coupling error.
     *
     * @return y-x cross coupling error.
     */
    public double getMyx() {
        return mCrossCouplingErrors.getElementAt(1, 0);
    }

    /**
     * Sets y-x cross coupling error.
     *
     * @param myx y-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMyx(final double myx) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(1, 0, myx);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets y-z cross coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getMyz() {
        return mCrossCouplingErrors.getElementAt(1, 2);
    }

    /**
     * Sets y-z cross coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMyz(final double myz) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(1, 2, myz);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets z-x cross coupling error.
     *
     * @return z-x cross coupling error.
     */
    public double getMzx() {
        return mCrossCouplingErrors.getElementAt(2, 0);
    }

    /**
     * Sets z-x cross coupling error.
     *
     * @param mzx z-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMzx(final double mzx) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(2, 0, mzx);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets z-y cross coupling error.
     *
     * @return z-y cross coupling error.
     */
    public double getMzy() {
        return mCrossCouplingErrors.getElementAt(2, 1);
    }

    /**
     * Sets z-y cross coupling error.
     *
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMzy(final double mzy) throws AlgebraException {
        final Matrix m = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS,
                MagneticFluxDensityTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(2, 1, mzy);
        setCrossCouplingErrors(m);
    }

    /**
     * Sets scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setScalingFactors(
            final double sx, final double sy, final double sz)
            throws AlgebraException {
        final Matrix m = new Matrix(
                AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(0, 0, sx);
        m.setElementAt(1, 1, sy);
        m.setElementAt(2, 2, sz);
        setCrossCouplingErrors(m);
    }

    /**
     * Sets cross coupling errors.
     *
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {
        final Matrix m = new Matrix(
                AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(0, 1, mxy);
        m.setElementAt(0, 2, mxz);
        m.setElementAt(1, 0, myx);
        m.setElementAt(1, 2, myz);
        m.setElementAt(2, 0, mzx);
        m.setElementAt(2, 1, mzy);
        setCrossCouplingErrors(m);
    }

    /**
     * Sets scaling factors and cross coupling errors.
     *
     * @param sx  x scaling factor.
     * @param sy  y scaling factor.
     * @param sz  z scaling factor.
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {
        final Matrix m = new Matrix(
                AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS);
        m.copyFrom(mCrossCouplingErrors);
        m.setElementAt(0, 0, sx);
        m.setElementAt(1, 1, sy);
        m.setElementAt(2, 2, sz);
        m.setElementAt(0, 1, mxy);
        m.setElementAt(0, 2, mxz);
        m.setElementAt(1, 0, myx);
        m.setElementAt(1, 2, myz);
        m.setElementAt(2, 0, mzx);
        m.setElementAt(2, 1, mzy);
        setCrossCouplingErrors(m);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if length of provided result array is not
     *                                  3.
     */
    public void fix(final BodyMagneticFluxDensity measuredB,
                    final double[] result) throws AlgebraException {
        if (result.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        fix(measuredB.getBx(), measuredB.getBy(), measuredB.getBz(), result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if length of provided result array is not
     *                                  3.
     */
    public void fix(final MagneticFluxDensityTriad measuredB,
                    final double[] result) throws AlgebraException {
        if (result.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        final double measuredBx = convertMagneticFluxDensity(measuredB.getValueX(),
                measuredB.getUnit());
        final double measuredBy = convertMagneticFluxDensity(measuredB.getValueY(),
                measuredB.getUnit());
        final double measuredBz = convertMagneticFluxDensity(measuredB.getValueZ(),
                measuredB.getUnit());
        fix(measuredBx, measuredBy, measuredBz, result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void fix(final BodyMagneticFluxDensity measuredB,
                    final Matrix result) throws AlgebraException {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredB.getBx(), measuredB.getBy(), measuredB.getBz(), result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void fix(final MagneticFluxDensityTriad measuredB,
                    final Matrix result) throws AlgebraException {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        final double measuredBx = convertMagneticFluxDensity(measuredB.getValueX(),
                measuredB.getUnit());
        final double measuredBy = convertMagneticFluxDensity(measuredB.getValueY(),
                measuredB.getUnit());
        final double measuredBz = convertMagneticFluxDensity(measuredB.getValueZ(),
                measuredB.getUnit());
        fix(measuredBx, measuredBy, measuredBz, result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final BodyMagneticFluxDensity measuredB,
                    final BodyMagneticFluxDensity result)
            throws AlgebraException {
        fix(measuredB, mResult);
        result.setCoordinates(mResult[0], mResult[1], mResult[2]);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final MagneticFluxDensityTriad measuredB,
                    final MagneticFluxDensityTriad result)
            throws AlgebraException {
        fix(measuredB, mResult);
        result.setValueCoordinates(mResult);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density.
     * @param measuredBy y-coordinate of measured magnetic flux density.
     * @param measuredBz z-coordinate of measured magnetic flux density.
     * @param result     instance where restored true magnetic flux density will be
     *                   stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final MagneticFluxDensity measuredBx,
                    final MagneticFluxDensity measuredBy,
                    final MagneticFluxDensity measuredBz,
                    final BodyMagneticFluxDensity result)
            throws AlgebraException {

        final double bx = convertMagneticFluxDensity(measuredBx);
        final double by = convertMagneticFluxDensity(measuredBy);
        final double bz = convertMagneticFluxDensity(measuredBz);
        fix(bx, by, bz, mResult);
        result.setCoordinates(mResult[0], mResult[1], mResult[2]);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density.
     * @param measuredBy y-coordinate of measured magnetic flux density.
     * @param measuredBz z-coordinate of measured magnetic flux density.
     * @param result     instance where restored true magnetic flux density will be
     *                   stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final MagneticFluxDensity measuredBx,
                    final MagneticFluxDensity measuredBy,
                    final MagneticFluxDensity measuredBz,
                    final MagneticFluxDensityTriad result)
            throws AlgebraException {

        final double bx = convertMagneticFluxDensity(measuredBx);
        final double by = convertMagneticFluxDensity(measuredBy);
        final double bz = convertMagneticFluxDensity(measuredBz);
        fix(bx, by, bz, mResult);
        result.setValueCoordinates(mResult);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured magnetic flux density expressed in Teslas (T).
     *                  Must have length 3.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final double[] measuredB, final double[] result)
            throws AlgebraException {
        if (measuredB.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The magnetometer model is
        // bmeas = bm + (I + Mm) * btrue

        // where:
        // bmeas is measured magnetic flux density (vector 3x1)
        // bm is magnetometer bias (vector 3x1)
        // I is the 3x3 identity
        // Mm is the 3x3 cross couplings matrix

        // Hecen:
        // btrue = (I + Mm)^-1 * (bmeas - bm)
        for (int i = 0; i < MagneticFluxDensityTriad.COMPONENTS; i++) {
            mDiff.setElementAtIndex(i,
                    measuredB[i] - mBias.getElementAtIndex(i));
        }

        mTmp2.multiply(mDiff, mTmp3);

        mTmp3.toArray(result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured magnetic flux density expressed in Teslas (T).
     *                  Must be 3x1.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredB, final double[] result)
            throws AlgebraException {

        if (measuredB.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || measuredB.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredB.getBuffer(), result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured magnetic flux density expressed in Teslas (T).
     *                  Must be 3x1.
     * @param result    instance where restored true magnetic flux density will be
     *                  stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredB, final Matrix result)
            throws AlgebraException {

        if (measuredB.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || measuredB.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredB, result.getBuffer());
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density expressed
     *                   in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density expressed
     *                   in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density expressed
     *                   in Teslas (T).
     * @param result     instance where restored true magnetic flux density will
     *                   be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided result array does not have
     *                                  length 3.
     */
    public void fix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double[] result) throws AlgebraException {

        mMeasuredB[0] = measuredBx;
        mMeasuredB[1] = measuredBy;
        mMeasuredB[2] = measuredBz;

        fix(mMeasuredB, result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density expressed
     *                   in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density expressed
     *                   in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density expressed
     *                   in Teslas (T).
     * @param result     instance where restored true magnetic flux density will
     *                   be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void fix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredBx, measuredBy, measuredBz, result.getBuffer());
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB           measured magnetic flux density expressed in
     *                            Teslas (T). Must have length 3.
     * @param bias                bias values expressed in Teslas (T). Must be
     *                            3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double[] measuredB, final Matrix bias,
            final Matrix crossCouplingErrors, final double[] result)
            throws AlgebraException {
        if (measuredB.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (bias.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        if (crossCouplingErrors.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || crossCouplingErrors.getColumns() != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != MagneticFluxDensityTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The magnetometer model is
        // bmeas = bm + (I + Mm) * btrue

        // where:
        // bmeas is measured specific force (vector 3x1)
        // ba is magnetometer bias (vector 3x1)
        // I is the 3x3 identity
        // Mm is the 3x3 cross couplings matrix

        // Hecen:
        // btrue = (I + Mm)^-1 * (bmeas - bm)
        mIdentity.add(crossCouplingErrors, mTmp1);

        Utils.inverse(mTmp1, mTmp2);

        for (int i = 0; i < MagneticFluxDensityTriad.COMPONENTS; i++) {
            mDiff.setElementAtIndex(i,
                    measuredB[i] - bias.getElementAtIndex(i));
        }

        mTmp2.multiply(mDiff, mTmp3);

        mTmp3.toArray(result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB           measured magnetic flux density expressed in
     *                            Teslas (T). Must be 3x1.
     * @param bias                bias values expressed in Teslas (T). Must be
     *                            3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final Matrix measuredB, final Matrix bias,
            final Matrix crossCouplingErrors, final double[] result)
            throws AlgebraException {

        if (measuredB.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || measuredB.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredB.getBuffer(), bias, crossCouplingErrors, result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB           measured magnetic flux density expressed in
     *                            Teslas (T). Must be 3x1.
     * @param bias                bias values expressed in Teslas (T). Must be
     *                            3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final Matrix measuredB, final Matrix bias,
            final Matrix crossCouplingErrors, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredB, bias, crossCouplingErrors, result.getBuffer());
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx          x-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBy          y-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBz          z-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param biasX               x-coordinate of bias expressed in Teslas (T).
     * @param biasY               y-coordinate of bias expressed in Teslas (T).
     * @param biasZ               z-coordinate of bias expressed in Teslas (T).
     * @param crossCouplingErrors cross-coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true magnetic flux
     *                            density will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors,
            final double[] result) throws AlgebraException {

        mMeasuredB[0] = measuredBx;
        mMeasuredB[1] = measuredBy;
        mMeasuredB[2] = measuredBz;

        mBias.setElementAtIndex(0, biasX);
        mBias.setElementAtIndex(1, biasY);
        mBias.setElementAtIndex(2, biasZ);

        fix(mMeasuredB, mBias, crossCouplingErrors, result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx          x-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBy          y-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBz          z-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param biasX               x-coordinate of bias expressed in Teslas (T).
     * @param biasY               y-coordinate of bias expressed in Teslas (T).
     * @param biasZ               z-coordinate of bias expressed in Teslas (T).
     * @param crossCouplingErrors cross-coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true magnetic flux
     *                            density will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != MagneticFluxDensityTriad.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ, crossCouplingErrors,
                result.getBuffer());
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param biasX      x-coordinate of bias expressed in Teslas (T).
     * @param biasY      y-coordinate of bias expressed in Teslas (T).
     * @param biasZ      z-coordinate of bias expressed in Teslas (T).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @param result     instance where restored true magnetic flux
     *                   density will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if result does not have length 3.
     */
    public void fix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy,
            final double[] result) throws AlgebraException {

        mCrossCouplingErrors.setElementAt(0, 0, sx);
        mCrossCouplingErrors.setElementAt(1, 1, sy);
        mCrossCouplingErrors.setElementAt(2, 2, sz);
        mCrossCouplingErrors.setElementAt(0, 1, mxy);
        mCrossCouplingErrors.setElementAt(0, 2, mxz);
        mCrossCouplingErrors.setElementAt(1, 0, myx);
        mCrossCouplingErrors.setElementAt(1, 2, myz);
        mCrossCouplingErrors.setElementAt(2, 0, mzx);
        mCrossCouplingErrors.setElementAt(2, 1, mzy);

        fix(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ,
                mCrossCouplingErrors, result);
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param biasX      x-coordinate of bias expressed in Teslas (T).
     * @param biasY      y-coordinate of bias expressed in Teslas (T).
     * @param biasZ      z-coordinate of bias expressed in Teslas (T).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @param result     instance where restored true magnetic flux
     *                   density will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if result is not 3x1.
     */
    public void fix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result.getBuffer());
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured body magnetic flux density.
     * @return restored true magnetic flux density.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public MagneticFluxDensityTriad fixAndReturnNew(
            final MagneticFluxDensityTriad measuredB) throws AlgebraException {
        final MagneticFluxDensityTriad result = new MagneticFluxDensityTriad();
        fix(measuredB, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density.
     * @param measuredBy y-coordinate of measured magnetic flux density.
     * @param measuredBz z-coordinate of measured magnetic flux density.
     * @return restored true magnetic flux density.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public MagneticFluxDensityTriad fixAndReturnNew(
            final MagneticFluxDensity measuredBx,
            final MagneticFluxDensity measuredBy,
            final MagneticFluxDensity measuredBz) throws AlgebraException {
        final MagneticFluxDensityTriad result = new MagneticFluxDensityTriad();
        fix(measuredBx, measuredBy, measuredBz, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured magnetic flux density expressed in Teslas (T).
     *                  Must have length 3.
     * @return restored true magnetic flux density.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public double[] fixAndReturnNew(
            final double[] measuredB) throws AlgebraException {

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredB, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured magnetic flux density expressed in Teslas (T).
     *                  Must be 3x1.
     * @return restored true magnetic flux density.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public double[] fixAndReturnNew(final Matrix measuredB)
            throws AlgebraException {

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredB, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB measured magnetic flux density expressed in Teslas (T).
     *                  Must be 3x1.
     * @return restored true magnetic flux density.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public Matrix fixAndReturnNewMatrix(final Matrix measuredB)
            throws AlgebraException {

        final Matrix result = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS, 1);
        fix(measuredB, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @return restored true magnetic flux density.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredBx, final double measuredBy, final double measuredBz)
            throws AlgebraException {
        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredBx, measuredBy, measuredBz, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @return restored true magnetic flux density.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredBx, final double measuredBy, final double measuredBz)
            throws AlgebraException {
        final Matrix result = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS, 1);
        fix(measuredBx, measuredBy, measuredBz, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB           measured magnetic flux density expressed in
     *                            Teslas (T). Must have length 3.
     * @param bias                bias values expressed in Teslas (T). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double[] measuredB, final Matrix bias,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredB, bias, crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB           measured magnetic flux density expressed in
     *                            Teslas (T). Must be 3x1.
     * @param bias                bias values expressed in Teslas (T). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final Matrix measuredB, final Matrix bias,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredB, bias, crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredB           measured magnetic flux density expressed in
     *                            Teslas (T). Must be 3x1.
     * @param bias                bias values expressed in Teslas (T). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final Matrix measuredB, final Matrix bias,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final Matrix result = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS, 1);
        fix(measuredB, bias, crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx          x-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBy          y-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBz          z-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param biasX               x-coordinate of bias expressed in Teslas (T).
     * @param biasY               y-coordinate of bias expressed in Teslas (T).
     * @param biasZ               z-coordinate of bias expressed in Teslas (T).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredBx, measuredBy, measuredBz,
                biasX, biasY, biasZ, crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx          x-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBy          y-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param measuredBz          z-coordinate of measured magnetic flux density
     *                            expressed in Teslas (T).
     * @param biasX               x-coordinate of bias expressed in Teslas (T).
     * @param biasY               y-coordinate of bias expressed in Teslas (T).
     * @param biasZ               z-coordinate of bias expressed in Teslas (T).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final Matrix result = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS, 1);
        fix(measuredBx, measuredBy, measuredBz,
                biasX, biasY, biasZ,
                crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param biasX      x-coordinate of bias expressed in Teslas (T).
     * @param biasY      y-coordinate of bias expressed in Teslas (T).
     * @param biasZ      z-coordinate of bias expressed in Teslas (T).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {

        final double[] result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fix(measuredBx, measuredBy, measuredBz,
                biasX, biasY, biasZ,
                sx, sy, sz,
                mxy, mxz,
                myx, myz,
                mzx, mzy, result);
        return result;
    }

    /**
     * Fixes provided measured body magnetic flux density values by undoing the
     * errors introduced by the magnetometer model to restore the true body
     * magnetic flux density.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredBx x-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBy y-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param measuredBz z-coordinate of measured magnetic flux density
     *                   expressed in Teslas (T).
     * @param biasX      x-coordinate of bias expressed in Teslas (T).
     * @param biasY      y-coordinate of bias expressed in Teslas (T).
     * @param biasZ      z-coordinate of bias expressed in Teslas (T).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @return restored true magnetic flux density expressed in Teslas (T).
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredBx, final double measuredBy, final double measuredBz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {

        final Matrix result = new Matrix(
                MagneticFluxDensityTriad.COMPONENTS, 1);
        fix(measuredBx, measuredBy, measuredBz,
                biasX, biasY, biasZ,
                sx, sy, sz,
                mxy, mxz,
                myx, myz,
                mzx, mzy, result);
        return result;
    }

    /**
     * Converts magnetic flux density value and unit to Teslas (T).
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    private static double convertMagneticFluxDensity(
            final double value, final MagneticFluxDensityUnit unit) {
        return MagneticFluxDensityConverter.convert(value, unit,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Converts magnetic flux density to Teslas (T).
     *
     * @param b magnetic flux density to be converted.
     * @return converted value.
     */
    public static double convertMagneticFluxDensity(final MagneticFluxDensity b) {
        return convertMagneticFluxDensity(b.getValue().doubleValue(),
                b.getUnit());
    }
}
