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
            mTmp1 = new Matrix(BodyKinematics.COMPONENTS,
                    BodyKinematics.COMPONENTS);
            mTmp2 = new Matrix(BodyKinematics.COMPONENTS,
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
     * @throws AlgebraException         if there are numerical instabilities.
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
     * @throws AlgebraException         if there are numerical instabilities.
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
}
