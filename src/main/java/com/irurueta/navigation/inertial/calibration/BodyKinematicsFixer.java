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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AngularSpeed;

/**
 * Fixes body kinematics (acceleration + angular rate) values taking into
 * account provided biases, cross coupling errors and G-dependent errors.
 */
public class BodyKinematicsFixer {

    /**
     * Fixes specific force (acceleration) components of body kinematics measurements.
     */
    private final AccelerationFixer mAccelerationFixer = new AccelerationFixer();

    /**
     * Fixes angular rate components of body kinematics measurements.
     */
    private final AngularRateFixer mAngularRateFixer = new AngularRateFixer();

    /**
     * Contains measured acceleration to be reused.
     */
    private final AccelerationTriad mMeasuredAcceleration = new AccelerationTriad();

    /**
     * Contains measured angular speed to be reuded.
     */
    private final AngularSpeedTriad mMeasuredAngularSpeed = new AngularSpeedTriad();

    /**
     * Contains fixed acceleration to be reused.
     */
    private final AccelerationTriad mFixedAcceleration = new AccelerationTriad();

    /**
     * Contains fixed angular speed to be reused.
     */
    private final AngularSpeedTriad mFixedAngularSpeed = new AngularSpeedTriad();

    /**
     * Constructor.
     */
    public BodyKinematicsFixer() {
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public Matrix getAccelerationBias() {
        return mAccelerationFixer.getBias();
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBias(final Matrix result) {
        mAccelerationFixer.getBias(result);
    }

    /**
     * Sets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second.
     *             Must be 3x1.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setAccelerationBias(final Matrix bias) {
        mAccelerationFixer.setBias(bias);
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public double[] getAccelerationBiasArray() {
        return mAccelerationFixer.getBiasArray();
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAccelerationBiasArray(final double[] result) {
        mAccelerationFixer.getBiasArray(result);
    }

    /**
     * Sets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second (m/s^2).
     *             Must have length 3.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setAccelerationBias(final double[] bias) {
        mAccelerationFixer.setBias(bias);
    }

    /**
     * Gets acceleration bias.
     *
     * @return acceleration bias.
     */
    public AccelerationTriad getAccelerationBiasAsTriad() {
        return mAccelerationFixer.getBiasAsTriad();
    }

    /**
     * Gets acceleration bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasAsTriad(final AccelerationTriad result) {
        mAccelerationFixer.getBiasAsTriad(result);
    }

    /**
     * Sets acceleration bias.
     *
     * @param bias acceleration bias to be set.
     */
    public void setAccelerationBias(final AccelerationTriad bias) {
        mAccelerationFixer.setBias(bias);
    }

    /**
     * Gets acceleration x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return x-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasX() {
        return mAccelerationFixer.getBiasX();
    }

    /**
     * Sets acceleration x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x-coordinate of bias expressed in meters per squared second
     *              (m/s^2).
     */
    public void setAccelerationBiasX(final double biasX) {
        mAccelerationFixer.setBiasX(biasX);
    }

    /**
     * Gets acceleration y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return y-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasY() {
        return mAccelerationFixer.getBiasY();
    }

    /**
     * Sets acceleration y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY y-coordinate of bias expressed in meters per squared second
     *              (m/s^2).
     */
    public void setAccelerationBiasY(final double biasY) {
        mAccelerationFixer.setBiasY(biasY);
    }

    /**
     * Gets acceleration z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return z-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasZ() {
        return mAccelerationFixer.getBiasZ();
    }

    /**
     * Sets acceleration z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ z-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasZ(final double biasZ) {
        mAccelerationFixer.setBiasZ(biasZ);
    }

    /**
     * Sets acceleration coordinates of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setAccelerationBias(
            final double biasX, final double biasY, final double biasZ) {
        mAccelerationFixer.setBias(biasX, biasY, biasZ);
    }

    /**
     * Gets acceleration x-coordinate of bias.
     *
     * @return acceleration x-coordinate of bias.
     */
    public Acceleration getAccelerationBiasXAsAcceleration() {
        return mAccelerationFixer.getBiasXAsAcceleration();
    }

    /**
     * Gets acceleration x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasXAsAcceleration(final Acceleration result) {
        mAccelerationFixer.getBiasXAsAcceleration(result);
    }

    /**
     * Sets acceleration x-coordinate of bias.
     *
     * @param biasX acceleration x-coordinate of bias.
     */
    public void setAccelerationBiasX(final Acceleration biasX) {
        mAccelerationFixer.setBiasX(biasX);
    }

    /**
     * Gets acceleration y-coordinate of bias.
     *
     * @return acceleration y-coordinate of bias.
     */
    public Acceleration getAccelerationBiasYAsAcceleration() {
        return mAccelerationFixer.getBiasYAsAcceleration();
    }

    /**
     * Gets acceleration y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasYAsAcceleration(final Acceleration result) {
        mAccelerationFixer.getBiasYAsAcceleration(result);
    }

    /**
     * Sets acceleration y-coordinate of bias.
     *
     * @param biasY acceleration y-coordinate of bias.
     */
    public void setAccelerationBiasY(final Acceleration biasY) {
        mAccelerationFixer.setBiasY(biasY);
    }

    /**
     * Gets acceleration z-coordinate of bias.
     *
     * @return acceleration z-coordinate of bias.
     */
    public Acceleration getAccelerationBiasZAsAcceleration() {
        return mAccelerationFixer.getBiasZAsAcceleration();
    }

    /**
     * Gets acceleration z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationBiasZAsAcceleration(final Acceleration result) {
        mAccelerationFixer.getBiasZAsAcceleration(result);
    }

    /**
     * Sets acceleration z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     */
    public void setAccelerationBiasZ(final Acceleration biasZ) {
        mAccelerationFixer.setBiasZ(biasZ);
    }

    /**
     * Sets acceleration coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setAccelerationBias(
            final Acceleration biasX,
            final Acceleration biasY,
            final Acceleration biasZ) {
        mAccelerationFixer.setBias(biasX, biasY, biasZ);
    }

    /**
     * Gets acceleration cross coupling errors matrix.
     *
     * @return acceleration cross coupling errors matrix.
     */
    public Matrix getAccelerationCrossCouplingErrors() {
        return mAccelerationFixer.getCrossCouplingErrors();
    }

    /**
     * Gets acceleration cross coupling errors matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getAccelerationCrossCouplingErrors(final Matrix result) {
        mAccelerationFixer.getCrossCouplingErrors(result);
    }

    /**
     * Sets acceleration cross coupling errors matrix.
     *
     * @param crossCouplingErrors acceleration cross coupling errors matrix.
     *                            Must be 3x3.
     * @throws AlgebraException         if provided matrix cannot be inverted.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setAccelerationCrossCouplingErrors(
            final Matrix crossCouplingErrors) throws AlgebraException {
        mAccelerationFixer.setCrossCouplingErrors(crossCouplingErrors);
    }

    /**
     * Gets acceleration x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getAccelerationSx() {
        return mAccelerationFixer.getSx();
    }

    /**
     * Sets acceleration x scaling factor.
     *
     * @param sx x scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationSx(final double sx) throws AlgebraException {
        mAccelerationFixer.setSx(sx);
    }

    /**
     * Gets acceleration y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getAccelerationSy() {
        return mAccelerationFixer.getSy();
    }

    /**
     * Sets acceleration y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationSy(final double sy) throws AlgebraException {
        mAccelerationFixer.setSy(sy);
    }

    /**
     * Gets acceleration z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getAccelerationSz() {
        return mAccelerationFixer.getSz();
    }

    /**
     * Sets acceleration z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationSz(final double sz) throws AlgebraException {
        mAccelerationFixer.setSz(sz);
    }

    /**
     * Gets acceleration x-y cross coupling error.
     *
     * @return acceleration x-y cross coupling error.
     */
    public double getAccelerationMxy() {
        return mAccelerationFixer.getMxy();
    }

    /**
     * Sets acceleration x-y cross coupling error.
     *
     * @param mxy acceleration x-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationMxy(final double mxy) throws AlgebraException {
        mAccelerationFixer.setMxy(mxy);
    }

    /**
     * Gets acceleration x-z cross coupling error.
     *
     * @return acceleration x-z cross coupling error.
     */
    public double getAccelerationMxz() {
        return mAccelerationFixer.getMxz();
    }

    /**
     * Sets acceleration x-z cross coupling error.
     *
     * @param mxz acceleration x-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationMxz(final double mxz) throws AlgebraException {
        mAccelerationFixer.setMxz(mxz);
    }

    /**
     * Gets acceleration y-x cross coupling error.
     *
     * @return acceleration y-x cross coupling error.
     */
    public double getAccelerationMyx() {
        return mAccelerationFixer.getMyx();
    }

    /**
     * Sets acceleration y-x cross coupling error.
     *
     * @param myx acceleration y-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationMyx(final double myx) throws AlgebraException {
        mAccelerationFixer.setMyx(myx);
    }

    /**
     * Gets acceleration y-z cross coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getAccelerationMyz() {
        return mAccelerationFixer.getMyz();
    }

    /**
     * Sets acceleration y-z cross coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationMyz(final double myz) throws AlgebraException {
        mAccelerationFixer.setMyz(myz);
    }

    /**
     * Gets acceleration z-x cross coupling error.
     *
     * @return acceleration z-x cross coupling error.
     */
    public double getAccelerationMzx() {
        return mAccelerationFixer.getMzx();
    }

    /**
     * Sets acceleration z-x cross coupling error.
     *
     * @param mzx acceleration z-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationMzx(final double mzx) throws AlgebraException {
        mAccelerationFixer.setMzx(mzx);
    }

    /**
     * Gets acceleration z-y cross coupling error.
     *
     * @return acceleration z-y cross coupling error.
     */
    public double getAccelerationMzy() {
        return mAccelerationFixer.getMzy();
    }

    /**
     * Sets acceleration z-y cross coupling error.
     *
     * @param mzy acceleration z-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationMzy(final double mzy) throws AlgebraException {
        mAccelerationFixer.setMzy(mzy);
    }

    /**
     * Sets acceleration scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setAccelerationScalingFactors(
            final double sx, final double sy, final double sz)
            throws AlgebraException {
        mAccelerationFixer.setScalingFactors(sx, sy, sz);
    }

    /**
     * Sets acceleration cross coupling errors.
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
    public void setAccelerationCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {
        mAccelerationFixer.setCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Sets acceleration scaling factors and cross coupling errors.
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
    public void setAccelerationScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {
        mAccelerationFixer.setScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @return angular speed bias values expressed in radians per second.
     */
    public Matrix getAngularSpeedBias() {
        return mAngularRateFixer.getBias();
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBias(final Matrix result) {
        mAngularRateFixer.getBias(result);
    }

    /**
     * Sets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second. Must be 3x1.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setAngularSpeedBias(final Matrix bias) {
        mAngularRateFixer.setBias(bias);
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @return bias values expressed in radians per second.
     */
    public double[] getAngularSpeedBiasArray() {
        return mAngularRateFixer.getBiasArray();
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAngularSpeedBiasArray(final double[] result) {
        mAngularRateFixer.getBiasArray(result);
    }

    /**
     * Sets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second (rad/s). Must
     *             have length 3.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void setAngularSpeedBias(final double[] bias) {
        mAngularRateFixer.setBias(bias);
    }

    /**
     * Gets angular speed bias.
     *
     * @return angular speed bias.
     */
    public AngularSpeedTriad getAngularSpeedBiasAsTriad() {
        return mAngularRateFixer.getBiasAsTriad();
    }

    /**
     * Gets angular speed bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasAsTriad(final AngularSpeedTriad result) {
        mAngularRateFixer.getBiasAsTriad(result);
    }

    /**
     * Sets angular speed bias.
     *
     * @param bias angular speed bias to be set.
     */
    public void setAngularSpeedBias(final AngularSpeedTriad bias) {
        mAngularRateFixer.setBias(bias);
    }

    /**
     * Gets angular speed x-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return x-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasX() {
        return mAngularRateFixer.getBiasX();
    }

    /**
     * Sets angular speed x-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x-coordinate of bias expressed in radians per second (rad/s).
     */
    public void setAngularSpeedBiasX(final double biasX) {
        mAngularRateFixer.setBiasX(biasX);
    }

    /**
     * Gets angular speed y-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return y-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasY() {
        return mAngularRateFixer.getBiasY();
    }

    /**
     * Sets angular speed y-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasY y-coordinate of bias expressed in radians per second (rad/s).
     */
    public void setAngularSpeedBiasY(final double biasY) {
        mAngularRateFixer.setBiasY(biasY);
    }

    /**
     * Gets angular speed z-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return z-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasZ() {
        return mAngularRateFixer.getBiasZ();
    }

    /**
     * Sets angular speed z-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z-coordinate of bias expressed in radians per second (rad/s).
     */
    public void setAngularSpeedBiasZ(final double biasZ) {
        mAngularRateFixer.setBiasZ(biasZ);
    }

    /**
     * Sets angular speed coordinates of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setAngularSpeedBias(
            final double biasX, final double biasY, final double biasZ) {
        mAngularRateFixer.setBias(biasX, biasY, biasZ);
    }

    /**
     * Gets angular speed x-coordinate of bias.
     *
     * @return x-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasXAsAngularSpeed() {
        return mAngularRateFixer.getBiasXAsAngularSpeed();
    }

    /**
     * Gets angular speed x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasXAsAngularSpeed(
            final AngularSpeed result) {
        mAngularRateFixer.getBiasXAsAngularSpeed(result);
    }

    /**
     * Sets angular speed x-coordinate of bias.
     *
     * @param biasX x-coordinate of bias.
     */
    public void setAngularSpeedBiasX(final AngularSpeed biasX) {
        mAngularRateFixer.setBiasX(biasX);
    }

    /**
     * Gets angular speed y-coordinate of bias.
     *
     * @return y-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasYAsAngularSpeed() {
        return mAngularRateFixer.getBiasYAsAngularSpeed();
    }

    /**
     * Gets angular speed y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasYAsAngularSpeed(final AngularSpeed result) {
        mAngularRateFixer.getBiasYAsAngularSpeed(result);
    }

    /**
     * Sets angular speed y-coordinate of bias.
     *
     * @param biasY y-coordinate of bias.
     */
    public void setAngularSpeedBiasY(final AngularSpeed biasY) {
        mAngularRateFixer.setBiasY(biasY);
    }

    /**
     * Gets angular speed z-coordinate of bias.
     *
     * @return z-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasZAsAngularSpeed() {
        return mAngularRateFixer.getBiasZAsAngularSpeed();
    }

    /**
     * Gets angular speed z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedBiasZAsAngularSpeed(final AngularSpeed result) {
        mAngularRateFixer.getBiasZAsAngularSpeed(result);
    }

    /**
     * Sets angular speed z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     */
    public void setAngularSpeedBiasZ(final AngularSpeed biasZ) {
        mAngularRateFixer.setBiasZ(biasZ);
    }

    /**
     * Sets angular speed coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setAngularSpeedBias(
            final AngularSpeed biasX,
            final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        mAngularRateFixer.setBias(biasX, biasY, biasZ);
    }

    /**
     * Gets angular speed cross coupling errors matrix.
     *
     * @return cross coupling errors matrix.
     */
    public Matrix getAngularSpeedCrossCouplingErrors() {
        return mAngularRateFixer.getCrossCouplingErrors();
    }

    /**
     * Gets angular speed cross coupling errors matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedCrossCouplingErrors(final Matrix result) {
        mAngularRateFixer.getCrossCouplingErrors(result);
    }

    /**
     * Sets angular speed cross coupling errors matrix.
     *
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided matrix cannot be inverted.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setAngularSpeedCrossCouplingErrors(
            final Matrix crossCouplingErrors) throws AlgebraException {
        mAngularRateFixer.setCrossCouplingErrors(crossCouplingErrors);
    }

    /**
     * Gets angular speed x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getAngularSpeedSx() {
        return mAngularRateFixer.getSx();
    }

    /**
     * sets angular speed x scaling factor.
     *
     * @param sx x scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedSx(final double sx) throws AlgebraException {
        mAngularRateFixer.setSx(sx);
    }

    /**
     * Gets angular speed y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getAngularSpeedSy() {
        return mAngularRateFixer.getSy();
    }

    /**
     * Sets angular speed y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedSy(final double sy) throws AlgebraException {
        mAngularRateFixer.setSy(sy);
    }

    /**
     * Gets angular speed z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getAngularSpeedSz() {
        return mAngularRateFixer.getSz();
    }

    /**
     * Sets angular speed z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedSz(final double sz) throws AlgebraException {
        mAngularRateFixer.setSz(sz);
    }

    /**
     * Gets angular speed x-y cross coupling error.
     *
     * @return x-y cross coupling error.
     */
    public double getAngularSpeedMxy() {
        return mAngularRateFixer.getMxy();
    }

    /**
     * Sets angular speed x-y cross coupling error.
     *
     * @param mxy x-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedMxy(final double mxy) throws AlgebraException {
        mAngularRateFixer.setMxy(mxy);
    }

    /**
     * Gets angular speed x-z cross coupling error.
     *
     * @return x-z cross coupling error.
     */
    public double getAngularSpeedMxz() {
        return mAngularRateFixer.getMxz();
    }

    /**
     * Sets angular speed x-z cross coupling error.
     *
     * @param mxz x-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedMxz(final double mxz) throws AlgebraException {
        mAngularRateFixer.setMxz(mxz);
    }

    /**
     * Gets angular speed y-x cross coupling error.
     *
     * @return y-x cross coupling error.
     */
    public double getAngularSpeedMyx() {
        return mAngularRateFixer.getMyx();
    }

    /**
     * Sets angular speed y-x cross coupling error.
     *
     * @param myx y-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedMyx(final double myx) throws AlgebraException {
        mAngularRateFixer.setMyx(myx);
    }

    /**
     * Gets angular speed y-z cross coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getAngularSpeedMyz() {
        return mAngularRateFixer.getMyz();
    }

    /**
     * Sets angular speed y-z cross coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedMyz(final double myz) throws AlgebraException {
        mAngularRateFixer.setMyz(myz);
    }

    /**
     * Gets angular speed z-x cross coupling error.
     *
     * @return z-x cross coupling error.
     */
    public double getAngularSpeedMzx() {
        return mAngularRateFixer.getMzx();
    }

    /**
     * Sets angular speed z-x cross coupling error.
     *
     * @param mzx z-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedMzx(final double mzx) throws AlgebraException {
        mAngularRateFixer.setMzx(mzx);
    }

    /**
     * Gets angular speed z-y cross coupling error.
     *
     * @return z-y cross coupling error.
     */
    public double getAngularSpeedMzy() {
        return mAngularRateFixer.getMzy();
    }

    /**
     * Sets angular speed z-y cross coupling error.
     *
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedMzy(final double mzy) throws AlgebraException {
        mAngularRateFixer.setMzy(mzy);
    }

    /**
     * Sets angular speed scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setAngularSpeedScalingFactors(
            final double sx, final double sy, final double sz)
            throws AlgebraException {
        mAngularRateFixer.setScalingFactors(sx, sy, sz);
    }

    /**
     * Sets angular speed cross coupling errors.
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
    public void setAngularSpeedCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy)
            throws AlgebraException {
        mAngularRateFixer.setCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Sets angular speed scaling factors and cross coupling errors.
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
    public void setAngularSpeedScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws AlgebraException {
        mAngularRateFixer.setScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Gets angular speed g-dependant cross biases matrix.
     *
     * @return g-dependant cross biases matrix.
     */
    public Matrix getAngularSpeedGDependantCrossBias() {
        return mAngularRateFixer.getGDependantCrossBias();
    }

    /**
     * Gets angular speed g-dependant cross biases matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getAngularSpeedGDependantCrossBias(final Matrix result) {
        mAngularRateFixer.getGDependantCrossBias(result);
    }

    /**
     * Sets angular speed g-dependant cross biases matrix.
     *
     * @param gDependantCrossBias g-dependant cross biases matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setAngularSpeedGDependantCrossBias(final Matrix gDependantCrossBias) {
        mAngularRateFixer.setGDependantCrossBias(gDependantCrossBias);
    }

    /**
     * Fixes provided measured body kinematics by undoing the errors introduced
     * by the accelerometer and gyroscope models to restore the true body
     * kinematics values.
     * This method uses last provided accelerometer and gyroscope bias and
     * cross coupling errors.
     *
     * @param measuredKinematics measured body kinematics to be fixed.
     * @param result             instance where fixed body kinematics will be
     *                           stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final BodyKinematics measuredKinematics,
                    final BodyKinematics result) throws AlgebraException {
        measuredKinematics.getSpecificForceTriad(mMeasuredAcceleration);
        measuredKinematics.getAngularRateTriad(mMeasuredAngularSpeed);

        fix(mMeasuredAcceleration, mMeasuredAngularSpeed,
                mFixedAcceleration, mFixedAngularSpeed);

        result.setSpecificForceTriad(mFixedAcceleration);
        result.setAngularRateTriad(mFixedAngularSpeed);
    }

    /**
     * Fixes provided measured body kinematics by undoing the errors introduced
     * by the accelerometer and gyroscope models to restore the true
     * body kinematics values.
     * This method uses last provided accelerometer and gyroscope bias and
     * cross coupling errors.
     *
     * @param measuredSpecificForce measured specific force to be fixed.
     * @param measuredAngularSpeed  measured angular speed to be fixed.
     * @param fixedSpecificForce    instance where fixed specific force will be
     *                              stored.
     * @param fixedAngularSpeed     instance where fixed angular speed will be
     *                              stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final AccelerationTriad measuredSpecificForce,
                    final AngularSpeedTriad measuredAngularSpeed,
                    final AccelerationTriad fixedSpecificForce,
                    final AngularSpeedTriad fixedAngularSpeed)
            throws AlgebraException {

        mAccelerationFixer.fix(measuredSpecificForce, fixedSpecificForce);
        mAngularRateFixer.fix(measuredAngularSpeed, fixedSpecificForce,
                fixedAngularSpeed);
    }

    /**
     * Fixes provided measured body kinematics by undoing the errors introduced
     * by the accelerometer and gyroscope models to restore the true
     * body kinematics values.
     * This method uses last provided accelerometer and gyroscope bias and
     * cross coupling errors.
     *
     * @param measuredKinematics measured body kinematics to be fixed.
     * @param fixedSpecificForce instance where fixed specific force will be
     *                           stored.
     * @param fixedAngularSpeed  instance where fixed angular speed will be
     *                           stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final BodyKinematics measuredKinematics,
                    final AccelerationTriad fixedSpecificForce,
                    final AngularSpeedTriad fixedAngularSpeed)
            throws AlgebraException {
        measuredKinematics.getSpecificForceTriad(mMeasuredAcceleration);
        measuredKinematics.getAngularRateTriad(mMeasuredAngularSpeed);

        fix(mMeasuredAcceleration, mMeasuredAngularSpeed,
                fixedSpecificForce, fixedAngularSpeed);
    }

    /**
     * Fixes provided measured body kinematics by undoing the errors introduced
     * by the accelerometer and gyroscope models to restore the true
     * body kinematics values.
     * This method uses last provided accelerometer and gyroscope bias and
     * cross coupling errors.
     *
     * @param measuredSpecificForce measured specific force to be fixed.
     * @param measuredAngularSpeed  measured angular speed to be fixed.
     * @param result                instance where fixed body kinematics will be
     *                              stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final AccelerationTriad measuredSpecificForce,
                    final AngularSpeedTriad measuredAngularSpeed,
                    final BodyKinematics result) throws AlgebraException {
        fix(measuredSpecificForce, measuredAngularSpeed,
                mFixedAcceleration, mFixedAngularSpeed);

        result.setSpecificForceTriad(mFixedAcceleration);
        result.setAngularRateTriad(mFixedAngularSpeed);
    }

    /**
     * Fixes provided measured body kinematics by undoing the errors introduced
     * by the accelerometer and gyroscope models to restore the true body
     * kinematics values.
     * This method uses last provided accelerometer and gyroscope bias and
     * cross coupling errors.
     *
     * @param measuredKinematics measured body kinematics to be fixed.
     * @return restored true body kinematics.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public BodyKinematics fixAndReturnNew(
            final BodyKinematics measuredKinematics) throws AlgebraException {
        final BodyKinematics result = new BodyKinematics();
        fix(measuredKinematics, result);
        return result;
    }

    /**
     * Fixes provided measured body kinematics by undoing the errors introduced
     * by the accelerometer and gyroscope models to restore the true body
     * kinematics values.
     * This method uses last provided accelerometer and gyroscope bias and
     * cross coupling errors.
     *
     * @param measuredSpecificForce measured specific force to be fixed.
     * @param measuredAngularSpeed  measured angular speed to be fixed.
     * @return restored true body kinematics.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public BodyKinematics fixAndReturnNew(
            final AccelerationTriad measuredSpecificForce,
            final AngularSpeedTriad measuredAngularSpeed)
            throws AlgebraException {
        final BodyKinematics result = new BodyKinematics();
        fix(measuredSpecificForce, measuredAngularSpeed, result);
        return result;
    }
}
