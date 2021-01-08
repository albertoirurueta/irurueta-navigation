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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

/**
 * Fixes angular rate values taking into
 * account provided bias, cross coupling errors and G-dependant errors.
 */
public class AngularRateFixer {
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
    private Matrix mTmp3;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mTmp4;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mDiff;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix mTmp5;

    /**
     * Measured angular rate to be reused.
     */
    private final double[] mMeasuredAngularRate =
            new double[BodyKinematics.COMPONENTS];

    /**
     * True specific force to be reused.
     */
    private final double[] mTrueF = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing result values to be reused.
     */
    private final double[] mResult = new double[BodyKinematics.COMPONENTS];

    /**
     * Bias matrix to be reused.
     */
    private Matrix mBias;

    /**
     * Cross coupling errors matrix to be reused.
     */
    private Matrix mCrossCouplingErrors;

    /**
     * G-dependant cross biases to be reused.
     */
    private Matrix mGDependantCrossBias;

    /**
     * Constructor.
     */
    public AngularRateFixer() {
        try {
            mIdentity = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mTmp1 = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mTmp2 = Matrix.identity(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mTmp3 = new Matrix(BodyKinematics.COMPONENTS, 1);
            mTmp4 = new Matrix(BodyKinematics.COMPONENTS, 1);
            mDiff = new Matrix(BodyKinematics.COMPONENTS, 1);
            mTmp5 = new Matrix(BodyKinematics.COMPONENTS, 1);

            mBias = new Matrix(BodyKinematics.COMPONENTS, 1);
            mCrossCouplingErrors = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mGDependantCrossBias = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets bias values expressed in radians per second (rad/s).
     *
     * @return bias values expressed in radians per second.
     */
    public Matrix getBias() {
        return new Matrix(mBias);
    }

    /**
     * Gets bias values expressed in radians per second (rad/s).
     *
     * @param result instance where result will be stored.
     */
    public void getBias(final Matrix result) {
        mBias.copyTo(result);
    }

    /**
     * Sets bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second. Must be 3x1.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setBias(final Matrix bias) {
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        mBias = bias;
    }

    /**
     * Gets bias values expressed in radians per second (rad/s).
     *
     * @return bias values expressed in radians per second.
     */
    public double[] getBiasArray() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBiasArray(result);
        return result;
    }

    /**
     * Gets bias values expressed in radians per second (rad/s).
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
     * Sets bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second (rad/s). Must
     *             have length 3.
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
     * Gets angular speed bias.
     *
     * @return angular speed bias.
     */
    public AngularSpeedTriad getBiasAsTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                mBias.getElementAtIndex(0), mBias.getElementAtIndex(1), mBias.getElementAtIndex(2));
    }

    /**
     * Gets angular speed bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AngularSpeedTriad result) {
        result.setValueCoordinates(mBias);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed bias.
     *
     * @param bias angular speed bias to be set.
     */
    public void setBias(final AngularSpeedTriad bias) {
        final double biasX = convertAngularSpeed(bias.getValueX(), bias.getUnit());
        final double biasY = convertAngularSpeed(bias.getValueY(), bias.getUnit());
        final double biasZ = convertAngularSpeed(bias.getValueZ(), bias.getUnit());
        mBias.setElementAtIndex(0, biasX);
        mBias.setElementAtIndex(1, biasY);
        mBias.setElementAtIndex(2, biasZ);
    }

    /**
     * Gets x-coordinate of bias expressed in radians per second (rad/s).
     *
     * @return x-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getBiasX() {
        return mBias.getElementAtIndex(0);
    }

    /**
     * Sets x-coordinate of bias expressed in radians per second (rad/s).
     *
     * @param biasX x-coordinate of bias expressed in radians per second
     *              (rad/s).
     */
    public void setBiasX(final double biasX) {
        mBias.setElementAtIndex(0, biasX);
    }

    /**
     * Gets y-coordinate of bias expressed in radians per second (rad/s).
     *
     * @return y-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getBiasY() {
        return mBias.getElementAtIndex(1);
    }

    /**
     * Sets y-coordinate of bias expressed in radians per second (rad/s).
     *
     * @param biasY y-coordinate of bias expressed in radians per second
     *              (rad/s).
     */
    public void setBiasY(final double biasY) {
        mBias.setElementAtIndex(1, biasY);
    }

    /**
     * Gets z-coordinate of bias expressed in radians per second (rad/s).
     *
     * @return z-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getBiasZ() {
        return mBias.getElementAtIndex(2);
    }

    /**
     * Sets z-coordinate of bias expressed in radians per second (rad/s).
     *
     * @param biasZ z-coordinate of bias expressed in radians per second
     *              (rad/s).
     */
    public void setBiasZ(final double biasZ) {
        mBias.setElementAtIndex(2, biasZ);
    }

    /**
     * Sets coordinates of bias expressed in radians per second (rad/s).
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
    public AngularSpeed getBiasXAsAngularSpeed() {
        return new AngularSpeed(getBiasX(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasXAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getBiasX());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets x-coordinate of bias.
     *
     * @param biasX x-coordinate of bias.
     */
    public void setBiasX(final AngularSpeed biasX) {
        setBiasX(convertAngularSpeed(biasX));
    }

    /**
     * Gets y-coordinate of bias.
     *
     * @return y-coordinate of bias.
     */
    public AngularSpeed getBiasYAsAngularSpeed() {
        return new AngularSpeed(getBiasY(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasYAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getBiasY());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets y-coordinate of bias.
     *
     * @param biasY y-coordinate of bias.
     */
    public void setBiasY(final AngularSpeed biasY) {
        setBiasY(convertAngularSpeed(biasY));
    }

    /**
     * Gets z-coordinate of bias.
     *
     * @return z-coordinate of bias.
     */
    public AngularSpeed getBiasZAsAngularSpeed() {
        return new AngularSpeed(getBiasZ(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasZAsAngularSpeed(final AngularSpeed result) {
        result.setValue(getBiasZ());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     */
    public void setBiasZ(final AngularSpeed biasZ) {
        setBiasZ(convertAngularSpeed(biasZ));
    }

    /**
     * Sets coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setBias(
            final AngularSpeed biasX,
            final AngularSpeed biasY,
            final AngularSpeed biasZ) {
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
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSx(final double sx) throws AlgebraException {
        final Matrix m = new Matrix(
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
            final double mzx, final double mzy)
            throws AlgebraException {
        final Matrix m = new Matrix(
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
                AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS);
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
     * Gets g-dependant cross biases matrix.
     *
     * @return g-dependant cross biases matrix.
     */
    public Matrix getGDependantCrossBias() {
        return new Matrix(mGDependantCrossBias);
    }

    /**
     * Gets g-dependant cross biases matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getGDependantCrossBias(final Matrix result) {
        mGDependantCrossBias.copyTo(result);
    }

    /**
     * Sets g-dependant cross biases matrix.
     *
     * @param gDependantCrossBias g-dependant cross biases matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setGDependantCrossBias(final Matrix gDependantCrossBias) {
        if (gDependantCrossBias.getRows() != BodyKinematics.COMPONENTS
                || gDependantCrossBias.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mGDependantCrossBias = gDependantCrossBias;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate.
     * @param trueF               true (i.e. fixed) specific force.
     * @param result              instance where restored true angular rate will
     *                            be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if length of provided result array is
     *                                  not 3.
     */
    public void fix(final AngularSpeedTriad measuredAngularRate,
                    final AccelerationTriad trueF,
                    final double[] result) throws AlgebraException {
        if (result.length != AngularSpeedTriad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        final double wX = convertAngularSpeed(measuredAngularRate.getValueX(),
                measuredAngularRate.getUnit());
        final double wY = convertAngularSpeed(measuredAngularRate.getValueY(),
                measuredAngularRate.getUnit());
        final double wZ = convertAngularSpeed(measuredAngularRate.getValueZ(),
                measuredAngularRate.getUnit());
        final double trueFx = convertAcceleration(trueF.getValueX(),
                trueF.getUnit());
        final double trueFy = convertAcceleration(trueF.getValueY(),
                trueF.getUnit());
        final double trueFz = convertAcceleration(trueF.getValueZ(),
                trueF.getUnit());
        fix(wX, wY, wZ, trueFx, trueFy, trueFz, result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate.
     * @param trueF               true (i.e. fixed) specific force.
     * @param result              instance where restored true angular rate will
     *                            be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if result matrix is not 3x1.
     */
    public void fix(final AngularSpeedTriad measuredAngularRate,
                    final AccelerationTriad trueF,
                    final Matrix result) throws AlgebraException {
        if (result.getRows() != AngularSpeedTriad.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRate, trueF, result.getBuffer());
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate.
     * @param trueF               true (i.e. fixed) specific force.
     * @param result              instance where restored true angular rate will be stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final AngularSpeedTriad measuredAngularRate,
                    final AccelerationTriad trueF,
                    final AngularSpeedTriad result) throws AlgebraException {
        fix(measuredAngularRate, trueF, mResult);
        result.setValueCoordinates(mResult);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular rate.
     * This method uses last provided and cross coupling errors.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate.
     * @param measuredAngularRateY y-coordinate of measured angular rate.
     * @param measuredAngularRateZ z-coordinate of measured angular rate.
     * @param trueFx               x-coordinate of true (i.e. fixed) specific force.
     * @param trueFy               y-coordinate of true (i.e. fixed) specific force.
     * @param trueFz               z-coordinate of true (i.e. fixed) specific force.
     * @param result               instance where restored true angular rate will be stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final AngularSpeed measuredAngularRateX,
                    final AngularSpeed measuredAngularRateY,
                    final AngularSpeed measuredAngularRateZ,
                    final Acceleration trueFx,
                    final Acceleration trueFy,
                    final Acceleration trueFz,
                    final AngularSpeedTriad result) throws AlgebraException {
        final double wX = convertAngularSpeed(measuredAngularRateX);
        final double wY = convertAngularSpeed(measuredAngularRateY);
        final double wZ = convertAngularSpeed(measuredAngularRateZ);
        final double fX = convertAcceleration(trueFx);
        final double fY = convertAcceleration(trueFy);
        final double fZ = convertAcceleration(trueFz);
        fix(wX, wY, wZ, fX, fY, fZ, mResult);
        result.setValueCoordinates(mResult);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must have length 3.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            have length 3.
     * @param result              instance where restored true angular rate
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double[] measuredAngularRate,
            final double[] trueF,
            final double[] result) throws AlgebraException {
        if (measuredAngularRate.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (trueF.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Ωtrue = (I + Mg)^-1 * (Ωmeas - bg - Gg * ftrue)

        mTmp3.fromArray(trueF);
        mGDependantCrossBias.multiply(mTmp3, mTmp4);

        for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
            mDiff.setElementAtIndex(i,
                    measuredAngularRate[i] - mBias.getElementAtIndex(i)
                            - mTmp4.getElementAtIndex(i));
        }

        mTmp2.multiply(mDiff, mTmp5);

        mTmp5.toArray(result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @param result              instance where restored true angular rate
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final Matrix measuredAngularRate,
            final Matrix trueF,
            final double[] result) throws AlgebraException {
        if (measuredAngularRate.getRows() != BodyKinematics.COMPONENTS
                || measuredAngularRate.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        if (trueF.getRows() != BodyKinematics.COMPONENTS
                || trueF.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRate.getBuffer(), trueF.getBuffer(), result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @param result              instance where restored true angular rate
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final Matrix measuredAngularRate,
            final Matrix trueF,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRate, trueF, result.getBuffer());
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param result               instance where restored true angular rate
     *                             will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double[] result) throws AlgebraException {

        mMeasuredAngularRate[0] = measuredAngularRateX;
        mMeasuredAngularRate[1] = measuredAngularRateY;
        mMeasuredAngularRate[2] = measuredAngularRateZ;

        mTrueF[0] = trueFx;
        mTrueF[1] = trueFy;
        mTrueF[2] = trueFz;

        fix(mMeasuredAngularRate, mTrueF, result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param result               instance where restored true angular rate
     *                             will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, result.getBuffer());
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must have length 3.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            have length 3.
     * @param bias                bias values expressed in radians per
     *                            second (rad/s). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias g-dependant cross biases matrix. Must be
     *                            3x3.
     * @param result              instance where restored true angular rate
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double[] measuredAngularRate,
            final double[] trueF,
            final Matrix bias,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias,
            final double[] result) throws AlgebraException {
        if (measuredAngularRate.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (trueF.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        if (crossCouplingErrors.getRows() != BodyKinematics.COMPONENTS
                || crossCouplingErrors.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (gDependantCrossBias.getRows() != BodyKinematics.COMPONENTS
                || gDependantCrossBias.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Ωtrue = (I + Mg)^-1 * (Ωmeas - bg - Gg * ftrue)
        mIdentity.add(crossCouplingErrors, mTmp1);

        Utils.inverse(mTmp1, mTmp2);

        mTmp3.fromArray(trueF);
        gDependantCrossBias.multiply(mTmp3, mTmp4);

        for (int i = 0; i < BodyKinematics.COMPONENTS; i++) {
            mDiff.setElementAtIndex(i,
                    measuredAngularRate[i] - bias.getElementAtIndex(i)
                            - mTmp4.getElementAtIndex(i));
        }

        mTmp2.multiply(mDiff, mTmp5);

        mTmp5.toArray(result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @param bias                bias values expressed in radians per
     *                            second (rad/s). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias g-dependant cross biases matrix. Must be
     *                            3x3.
     * @param result              instance where restored true angular rate
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final Matrix measuredAngularRate,
            final Matrix trueF,
            final Matrix bias,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias,
            final double[] result) throws AlgebraException {

        if (measuredAngularRate.getRows() != BodyKinematics.COMPONENTS
                || measuredAngularRate.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        if (trueF.getRows() != BodyKinematics.COMPONENTS
                || trueF.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRate.getBuffer(), trueF.getBuffer(),
                bias, crossCouplingErrors, gDependantCrossBias, result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @param bias                bias values expressed in radians per
     *                            second (rad/s). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias g-dependant cross biases matrix. Must be
     *                            3x3.
     * @param result              instance where restored true angular rate
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final Matrix measuredAngularRate,
            final Matrix trueF,
            final Matrix bias,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRate, trueF, bias, crossCouplingErrors,
                gDependantCrossBias, result.getBuffer());
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param crossCouplingErrors  cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias  g-dependant cross biases matrix. Must be
     *                             3x3.
     * @param result               instance where restored true angular rate
     *                             will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias,
            final double[] result) throws AlgebraException {

        mMeasuredAngularRate[0] = measuredAngularRateX;
        mMeasuredAngularRate[1] = measuredAngularRateY;
        mMeasuredAngularRate[2] = measuredAngularRateZ;

        mTrueF[0] = trueFx;
        mTrueF[1] = trueFy;
        mTrueF[2] = trueFz;

        mBias.setElementAtIndex(0, biasX);
        mBias.setElementAtIndex(1, biasY);
        mBias.setElementAtIndex(2, biasZ);

        fix(mMeasuredAngularRate, mTrueF, mBias, crossCouplingErrors,
                gDependantCrossBias, result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param crossCouplingErrors  cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias  g-dependant cross biases matrix. Must be
     *                             3x3.
     * @param result               instance where restored true angular rate
     *                             will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                crossCouplingErrors, gDependantCrossBias,
                result.getBuffer());
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param sx                   initial x scaling factor.
     * @param sy                   initial y scaling factor.
     * @param sz                   initial z scaling factor.
     * @param mxy                  initial x-y cross coupling error.
     * @param mxz                  initial x-z cross coupling error.
     * @param myx                  initial y-x cross coupling error.
     * @param myz                  initial y-z cross coupling error.
     * @param mzx                  initial z-x cross coupling error.
     * @param mzy                  initial z-y cross coupling error.
     * @param g11                  element 1,1 of g-dependant cross biases.
     * @param g21                  element 2,1 of g-dependant cross biases.
     * @param g31                  element 3,1 of g-dependant cross biases.
     * @param g12                  element 1,2 of g-dependant cross biases.
     * @param g22                  element 2,2 of g-dependant cross biases.
     * @param g32                  element 3,2 of g-dependant cross biases.
     * @param g13                  element 1,3 of g-dependant cross biases.
     * @param g23                  element 2,3 of g-dependant cross biases.
     * @param g33                  element 3,3 of g-dependant cross biases.
     * @param result               instance where restored true angular rate
     *                             will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy,
            final double g11, final double g21, final double g31,
            final double g12, final double g22, final double g32,
            final double g13, final double g23, final double g33,
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

        mGDependantCrossBias.setElementAt(0, 0, g11);
        mGDependantCrossBias.setElementAt(1, 0, g21);
        mGDependantCrossBias.setElementAt(2, 0, g31);
        mGDependantCrossBias.setElementAt(0, 1, g12);
        mGDependantCrossBias.setElementAt(1, 1, g22);
        mGDependantCrossBias.setElementAt(2, 1, g32);
        mGDependantCrossBias.setElementAt(0, 2, g13);
        mGDependantCrossBias.setElementAt(1, 2, g23);
        mGDependantCrossBias.setElementAt(2, 2, g33);

        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                mCrossCouplingErrors, mGDependantCrossBias, result);
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param sx                   initial x scaling factor.
     * @param sy                   initial y scaling factor.
     * @param sz                   initial z scaling factor.
     * @param mxy                  initial x-y cross coupling error.
     * @param mxz                  initial x-z cross coupling error.
     * @param myx                  initial y-x cross coupling error.
     * @param myz                  initial y-z cross coupling error.
     * @param mzx                  initial z-x cross coupling error.
     * @param mzy                  initial z-y cross coupling error.
     * @param g11                  element 1,1 of g-dependant cross biases.
     * @param g21                  element 2,1 of g-dependant cross biases.
     * @param g31                  element 3,1 of g-dependant cross biases.
     * @param g12                  element 1,2 of g-dependant cross biases.
     * @param g22                  element 2,2 of g-dependant cross biases.
     * @param g32                  element 3,2 of g-dependant cross biases.
     * @param g13                  element 1,3 of g-dependant cross biases.
     * @param g23                  element 2,3 of g-dependant cross biases.
     * @param g33                  element 3,3 of g-dependant cross biases.
     * @param result               instance where restored true angular rate
     *                             will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public void fix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy,
            final double g11, final double g21, final double g31,
            final double g12, final double g22, final double g32,
            final double g13, final double g23, final double g33,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS
                || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32, g13, g23, g33,
                result.getBuffer());
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate.
     * @param trueF               true (i.e. fixed) specific force.
     * @return restored true angular rate.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public AngularSpeedTriad fixAndReturnNew(
            final AngularSpeedTriad measuredAngularRate,
            final AccelerationTriad trueF) throws AlgebraException {
        final AngularSpeedTriad result = new AngularSpeedTriad();
        fix(measuredAngularRate, trueF, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate.
     * @param measuredAngularRateY y-coordinate of measured angular rate.
     * @param measuredAngularRateZ z-coordinate of measured angular rate.
     * @param trueFx               x-coordinate of true (i.e. fixed) specific force.
     * @param trueFy               y-coordinate of true (i.e. fixed) specific force.
     * @param trueFz               z-coordinate of true (i.e. fixed) specific force.
     * @return restored true angular rate.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public AngularSpeedTriad fixAndReturnNew(
            final AngularSpeed measuredAngularRateX,
            final AngularSpeed measuredAngularRateY,
            final AngularSpeed measuredAngularRateZ,
            final Acceleration trueFx,
            final Acceleration trueFy,
            final Acceleration trueFz) throws AlgebraException {
        final AngularSpeedTriad result = new AngularSpeedTriad();
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must have length 3.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            have length 3.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public double[] fixAndReturnNew(
            final double[] measuredAngularRate,
            final double[] trueF) throws AlgebraException {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRate, trueF, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public double[] fixAndReturnNew(
            final Matrix measuredAngularRate,
            final Matrix trueF) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRate, trueF, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final Matrix measuredAngularRate,
            final Matrix trueF) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredAngularRate, trueF, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @return restored true angular rate.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @return restored true angular rate.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, result);
        return result;
    }


    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must have length 3.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            have length 3.
     * @param bias                bias values expressed in radians per
     *                            second (rad/s). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias g-dependant cross biases matrix. Must be
     *                            3x3.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public double[] fixAndReturnNew(
            final double[] measuredAngularRate,
            final double[] trueF,
            final Matrix bias,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRate, trueF, bias, crossCouplingErrors,
                gDependantCrossBias, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @param bias                bias values expressed in radians per
     *                            second (rad/s). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias g-dependant cross biases matrix. Must be
     *                            3x3.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public double[] fixAndReturnNew(
            final Matrix measuredAngularRate,
            final Matrix trueF,
            final Matrix bias,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRate, trueF, bias, crossCouplingErrors,
                gDependantCrossBias, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRate measured angular rate expressed in radians
     *                            per second (rad/s). Must be 3x1.
     * @param trueF               true (i.e. fixed) specific force expressed
     *                            in meters per squared second (m/s^2). Must
     *                            be 3x1.
     * @param bias                bias values expressed in radians per
     *                            second (rad/s). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias g-dependant cross biases matrix. Must be
     *                            3x3.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final Matrix measuredAngularRate,
            final Matrix trueF,
            final Matrix bias,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredAngularRate, trueF, bias, crossCouplingErrors,
                gDependantCrossBias, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param crossCouplingErrors  cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias  g-dependant cross biases matrix. Must be
     *                             3x3.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public double[] fixAndReturnNew(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias) throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                crossCouplingErrors, gDependantCrossBias, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param crossCouplingErrors  cross coupling errors matrix. Must be 3x3.
     * @param gDependantCrossBias  g-dependant cross biases matrix. Must be
     *                             3x3.
     * @return restored true angular rate.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters
     *                                  does not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final Matrix crossCouplingErrors,
            final Matrix gDependantCrossBias) throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                crossCouplingErrors, gDependantCrossBias, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param sx                   initial x scaling factor.
     * @param sy                   initial y scaling factor.
     * @param sz                   initial z scaling factor.
     * @param mxy                  initial x-y cross coupling error.
     * @param mxz                  initial x-z cross coupling error.
     * @param myx                  initial y-x cross coupling error.
     * @param myz                  initial y-z cross coupling error.
     * @param mzx                  initial z-x cross coupling error.
     * @param mzy                  initial z-y cross coupling error.
     * @param g11                  element 1,1 of g-dependant cross biases.
     * @param g21                  element 2,1 of g-dependant cross biases.
     * @param g31                  element 3,1 of g-dependant cross biases.
     * @param g12                  element 1,2 of g-dependant cross biases.
     * @param g22                  element 2,2 of g-dependant cross biases.
     * @param g32                  element 3,2 of g-dependant cross biases.
     * @param g13                  element 1,3 of g-dependant cross biases.
     * @param g23                  element 2,3 of g-dependant cross biases.
     * @param g33                  element 3,3 of g-dependant cross biases.
     * @return restored true angular rate.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy,
            final double g11, final double g21, final double g31,
            final double g12, final double g22, final double g32,
            final double g13, final double g23, final double g33)
            throws AlgebraException {

        final double[] result = new double[BodyKinematics.COMPONENTS];
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32,
                g13, g23, g33, result);
        return result;
    }

    /**
     * Fixes provided measured angular rate values by undoing the errors
     * introduced by the gyroscope model to restore the true angular
     * rate.
     *
     * @param measuredAngularRateX x-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateY y-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param measuredAngularRateZ z-coordinate of measured angular rate
     *                             expressed in radians per second (rad/s).
     * @param trueFx               x-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFy               y-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param trueFz               z-coordinate of true (i.e. fixed)
     *                             specific force expressed in meters per
     *                             squared second (m/s^2).
     * @param biasX                x-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasY                y-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param biasZ                z-coordinate of bias expressed in
     *                             meters per squared second (m/s^2).
     * @param sx                   initial x scaling factor.
     * @param sy                   initial y scaling factor.
     * @param sz                   initial z scaling factor.
     * @param mxy                  initial x-y cross coupling error.
     * @param mxz                  initial x-z cross coupling error.
     * @param myx                  initial y-x cross coupling error.
     * @param myz                  initial y-z cross coupling error.
     * @param mzx                  initial z-x cross coupling error.
     * @param mzy                  initial z-y cross coupling error.
     * @param g11                  element 1,1 of g-dependant cross biases.
     * @param g21                  element 2,1 of g-dependant cross biases.
     * @param g31                  element 3,1 of g-dependant cross biases.
     * @param g12                  element 1,2 of g-dependant cross biases.
     * @param g22                  element 2,2 of g-dependant cross biases.
     * @param g32                  element 3,2 of g-dependant cross biases.
     * @param g13                  element 1,3 of g-dependant cross biases.
     * @param g23                  element 2,3 of g-dependant cross biases.
     * @param g33                  element 3,3 of g-dependant cross biases.
     * @return restored true angular rate.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredAngularRateX,
            final double measuredAngularRateY,
            final double measuredAngularRateZ,
            final double trueFx,
            final double trueFy,
            final double trueFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy,
            final double g11, final double g21, final double g31,
            final double g12, final double g22, final double g32,
            final double g13, final double g23, final double g33)
            throws AlgebraException {

        final Matrix result = new Matrix(
                BodyKinematics.COMPONENTS, 1);
        fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                g11, g21, g31, g12, g22, g32, g13, g23, g33,
                result);
        return result;
    }

    /**
     * Converts angular speed value and unit to radians per second (rad/s).
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(
            final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts angular speed measurement to radians per second (rad/s).
     *
     * @param angularSpeed angular speed to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return convertAngularSpeed(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit());
    }

    /**
     * Converts acceleration value and unit to meters per squared second (m/s^2).
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(
            final double value, final AccelerationUnit unit) {
        return AccelerationConverter.convert(value, unit,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts acceleration measurement to meters per squared second (m/s^2).
     *
     * @param acceleration acceleration to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(),
                acceleration.getUnit());
    }
}
