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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;

/**
 * Fixes acceleration values taking into account provided bias and cross coupling errors.
 */
public class AccelerationFixer {
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
    private final double[] mMeasuredF =
            new double[BodyKinematics.COMPONENTS];

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
    public AccelerationFixer() {
        try {
            mIdentity = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mTmp1 = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mTmp2 = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mDiff = new Matrix(BodyKinematics.COMPONENTS, 1);
            mTmp3 = new Matrix(BodyKinematics.COMPONENTS, 1);

            mBias = new Matrix(BodyKinematics.COMPONENTS, 1);
            mCrossCouplingErrors = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public Matrix getBias() {
        return new Matrix(mBias);
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result will be stored.
     */
    public void getBias(final Matrix result) {
        mBias.copyTo(result);
    }

    /**
     * Sets bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second.
     *             Must be 3x1.
     */
    public void setBias(final Matrix bias) {
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        mBias = bias;
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public double[] getBiasArray() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBiasArray(result);
        return result;
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void getBiasArray(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        try {
            mBias.toArray(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Sets bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second (m/s^2).
     *             Must have length 3.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void setBias(final double[] bias) {
        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        try {
            mBias.fromArray(bias);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets x-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @return x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     */
    public double getBiasX() {
        return mBias.getElementAtIndex(0);
    }

    /**
     * Sets x-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasX x-coordinate of bias expressed in meters per squared
     *              second (m/s^2).
     */
    public void setBiasX(final double biasX) {
        mBias.setElementAtIndex(0, biasX);
    }

    /**
     * Gets y-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @return y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     */
    public double getBiasY() {
        return mBias.getElementAtIndex(1);
    }

    /**
     * Sets y-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasY y-coordinate of bias expressed in meters per squared
     *              second (m/s^2).
     */
    public void setBiasY(final double biasY) {
        mBias.setElementAtIndex(1, biasY);
    }

    /**
     * Gets z-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @return z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     */
    public double getBiasZ() {
        return mBias.getElementAtIndex(2);
    }

    /**
     * Sets z-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasZ z-coordinate of bias expressed in meters per squared
     *              second (m/s^2).
     */
    public void setBiasZ(final double biasZ) {
        mBias.setElementAtIndex(2, biasZ);
    }

    /**
     * Sets coordinates of bias expressed in meters per squared second
     * (m/s^2).
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
        if (crossCouplingErrors.getRows() != BodyKinematics.COMPONENTS
                || crossCouplingErrors.getColumns() != BodyKinematics.COMPONENTS) {
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
     */
    public void setSx(final double sx) {
        mCrossCouplingErrors.setElementAt(0, 0, sx);
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
     */
    public void setSy(final double sy) {
        mCrossCouplingErrors.setElementAt(1, 1, sy);
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
     */
    public void setSz(final double sz) {
        mCrossCouplingErrors.setElementAt(2, 2, sz);
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
     */
    public void setMxy(final double mxy) {
        mCrossCouplingErrors.setElementAt(0, 1, mxy);
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
     */
    public void setMxz(final double mxz) {
        mCrossCouplingErrors.setElementAt(0, 2, mxz);
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
     */
    public void setMyx(final double myx) {
        mCrossCouplingErrors.setElementAt(1, 0, myx);
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
     */
    public void setMyz(final double myz) {
        mCrossCouplingErrors.setElementAt(1, 2, myz);
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
     */
    public void setMzx(final double mzx) {
        mCrossCouplingErrors.setElementAt(2, 0, mzx);
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
     */
    public void setMzy(final double mzy) {
        mCrossCouplingErrors.setElementAt(2, 1, mzy);
    }

    /**
     * Sets scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     */
    public void setScalingFactors(
            final double sx, final double sy, final double sz) {
        setSx(sx);
        setSy(sy);
        setSz(sz);
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
     */
    public void setCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) {
        setMxy(mxy);
        setMxz(mxz);
        setMyx(myx);
        setMyz(myz);
        setMzx(mzx);
        setMzy(mzy);
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
     */
    public void setScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) {
        setScalingFactors(sx, sy, sz);
        setCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must have length 3.
     * @param result    instance where restored true specific force will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final double[] measuredF, final double[] result)
            throws AlgebraException {
        if (measuredF.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The accelerometer model is
        // fmeas = ba + (I + Ma) * ftrue

        // where:
        // fmeas is measured specific force (vector 3x1)
        // ba is accelerometer bias (vector 3x1)
        // I is the 3x3 identity
        // Ma is the 3x3 cross couplings matrix

        // Hecen:
        // ftrue = (I + Ma)^-1 * (fmeas - ba)
        for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
            mDiff.setElementAtIndex(i,
                    measuredF[i] - mBias.getElementAtIndex(i));
        }

        mTmp2.multiply(mDiff, mTmp3);

        mTmp3.toArray(result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @param result    instance where restored true specific force will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredF, final double[] result)
            throws AlgebraException {

        if (measuredF.getRows() != BodyKinematics.COMPONENTS
                || measuredF.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF.getBuffer(), result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @param result    instance where restored true specific force will be
     *                  stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredF, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param result     instance where restored true specific force
     *                   will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double[] result) throws AlgebraException {

        mMeasuredF[0] = measuredFx;
        mMeasuredF[1] = measuredFy;
        mMeasuredF[2] = measuredFz;

        fix(mMeasuredF, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param result     instance where restored true specific force
     *                   will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredFx, measuredFy, measuredFz, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double[] measuredF, final Matrix bias,
            final Matrix crossCouplingErrors, final double[] result)
            throws AlgebraException {
        if (measuredF.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        if (crossCouplingErrors.getRows() != BodyKinematics.COMPONENTS
                || crossCouplingErrors.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The accelerometer model is
        // fmeas = ba + (I + Ma) * ftrue

        // where:
        // fmeas is measured specific force (vector 3x1)
        // ba is accelerometer bias (vector 3x1)
        // I is the 3x3 identity
        // Ma is the 3x3 cross couplings matrix

        // Hecen:
        // ftrue = (I + Ma)^-1 * (fmeas - ba)
        mIdentity.add(crossCouplingErrors, mTmp1);

        Utils.inverse(mTmp1, mTmp2);

        for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
            mDiff.setElementAtIndex(i,
                    measuredF[i] - bias.getElementAtIndex(i));
        }

        mTmp2.multiply(mDiff, mTmp3);

        mTmp3.toArray(result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must be 3x1.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final Matrix measuredF, final Matrix bias,
            final Matrix crossCouplingErrors, final double[] result)
            throws AlgebraException {

        if (measuredF.getRows() != BodyKinematics.COMPONENTS
                || measuredF.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF.getBuffer(), bias, crossCouplingErrors, result);
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final Matrix measuredF, final Matrix bias,
            final Matrix crossCouplingErrors, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF, bias, crossCouplingErrors, result.getBuffer());
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors,
            final double[] result) throws AlgebraException {

        mMeasuredF[0] = measuredFx;
        mMeasuredF[1] = measuredFy;
        mMeasuredF[2] = measuredFz;

        mBias.setElementAtIndex(0, biasX);
        mBias.setElementAtIndex(1, biasY);
        mBias.setElementAtIndex(2, biasZ);

        fix(mMeasuredF, mBias, crossCouplingErrors, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, crossCouplingErrors,
                result.getBuffer());
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @param result     instance where restored true specific force
     *                   will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
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

        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ,
                mCrossCouplingErrors, result);
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @param result     instance where restored true specific force
     *                   will be stored. Must have be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
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

        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must have length 3.
     * @return restored true specific force.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double[] measuredF) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @return restored true specific force.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(final Matrix measuredF)
            throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @return restored true specific force.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(final Matrix measuredF)
            throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @return restored true specific force.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredFx, final double measuredFy, final double measuredFz)
            throws AlgebraException {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredFx, measuredFy, measuredFz, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @return restored true specific force.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredFx, final double measuredFy, final double measuredFz)
            throws AlgebraException {
        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredFx, measuredFy, measuredFz, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double[] measuredF, final Matrix bias,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, bias, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final Matrix measuredF, final Matrix bias,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, bias, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final Matrix measuredF, final Matrix bias,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredF, bias, crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ,
                crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ,
                sx, sy, sz,
                mxy, mxz,
                myx, myz,
                mzx, mzy, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ,
                sx, sy, sz,
                mxy, mxz,
                myx, myz,
                mzx, mzy, result);
        return result;
    }
}
