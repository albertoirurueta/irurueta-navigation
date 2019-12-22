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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;

import java.util.Collection;

/**
 * Calculates position, velocity, clock offset and clock drift using
 * unweighted iterated least squares.
 * Separate calculations are implemented for position and clock offset and
 * for velocity and clock drift.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
 */
public class GNSSLeastSquaresPositionAndVelocityEstimator {

    /**
     * Minimum nuber of measurements required to obtain a solution.
     */
    public static final int MIN_MEASUREMENTS = 4;

    /**
     * Default threshold to determine when convergence has been reached.
     */
    public static final double CONVERGENCE_THRESHOLD = 1e-4;

    /**
     * Speed of light in the vacuum expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = Constants.SPEED_OF_LIGHT;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Number of components of predicted state.
     */
    private static final int STATE_COMPONENTS = ECEFPosition.COMPONENTS + 1;

    /**
     * Number of elements of position, velocity, etc.
     */
    private static final int ELEMS = ECEFPosition.COMPONENTS;

    /**
     * Number of elements of position minus one.
     */
    private static final int ELEMS_MINUS_ONE = ELEMS - 1;

    /**
     * GNSS measurements of a collection of satellites.
     */
    private Collection<GNSSMeasurement> mMeasurements;

    /**
     * Previously predicted ECEF user position and velocity.
     */
    private ECEFPositionAndVelocity mPriorPositionAndVelocity;

    /**
     * Listener to notify events raised by this instance.
     */
    private GNSSLeastSquaresPositionAndVelocityEstimatorListener mListener;

    /**
     * Threshold to determine when convergence has been reached.
     */
    private double mConvergenceThreshold = CONVERGENCE_THRESHOLD;

    /**
     * Indicates whether estimation is currently running.
     */
    private boolean mRunning;

    /**
     * Internal matrix to be reused containing frame rotation during signal transit
     * time.
     */
    private final Matrix mCei;

    /**
     * Predicted state to be reused.
     */
    private final Matrix mXPred;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp1;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp2;

    /**
     * Estimated state to be reused.
     */
    private final Matrix mXEst;

    /**
     * Contains square representation of measurement or geometry matrix.
     */
    private final Matrix mHSqr;

    /**
     * Inverse of the square representation of measurement or geometry matrix.
     */
    private final Matrix mInvHSqr;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp3;

    /**
     * Skew symmetric matrix of Earth rotation rate.
     */
    private final Matrix mOmegaIe;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp4;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp5;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp6;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp7;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp8;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp9;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix mTmp10;

    /**
     * Measurement position to be reused.
     */
    private final Matrix mMeasurementPosition;

    /**
     * Measurement velocity to be reused.
     */
    private final Matrix mMeasurementVelocity;

    /**
     * Predicted velocity to be reused.
     */
    private final Matrix mPredVelocity;

    /**
     * Result position to be reused.
     */
    private final Matrix mResultPosition;

    /**
     * Constructor.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator() {
        Matrix xPred = null;
        Matrix tmp1 = null;
        Matrix tmp2 = null;
        Matrix xEst = null;
        Matrix hSqr = null;
        Matrix invHSqr = null;
        Matrix tmp3 = null;
        Matrix omegaIe = null;
        Matrix cei = null;
        Matrix tmp4 = null;
        Matrix tmp5 = null;
        Matrix tmp6 = null;
        Matrix tmp7 = null;
        Matrix tmp8 = null;
        Matrix tmp9 = null;
        Matrix tmp10 = null;
        Matrix measurementPosition = null;
        Matrix measurementVelocity = null;
        Matrix predVelocity = null;
        Matrix resultPosition = null;
        try {
            cei = Matrix.identity(ELEMS, ELEMS);
            xPred = new Matrix(STATE_COMPONENTS, 1);
            tmp1 = new Matrix(ELEMS, 1);
            tmp2 = new Matrix(ELEMS, 1);
            xEst = new Matrix(STATE_COMPONENTS, 1);
            hSqr = new Matrix(STATE_COMPONENTS, STATE_COMPONENTS);
            invHSqr = new Matrix(STATE_COMPONENTS, STATE_COMPONENTS);
            tmp3 = new Matrix(STATE_COMPONENTS, 1);
            omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, EARTH_ROTATION_RATE});
            tmp4 = new Matrix(STATE_COMPONENTS, 1);
            tmp5 = new Matrix(STATE_COMPONENTS, 1);
            tmp6 = new Matrix(STATE_COMPONENTS, 1);
            tmp7 = new Matrix(STATE_COMPONENTS, 1);
            tmp8 = new Matrix(STATE_COMPONENTS, 1);
            tmp9 = new Matrix(STATE_COMPONENTS, 1);
            tmp10 = new Matrix(STATE_COMPONENTS, 1);
            measurementPosition = new Matrix(ELEMS, 1);
            measurementVelocity = new Matrix(ELEMS, 1);
            predVelocity = new Matrix(ELEMS, 1);
            resultPosition = new Matrix(ELEMS, 1);
        } catch (WrongSizeException ignore) {
            // never happens
        }

        mCei = cei;
        mXPred = xPred;
        mTmp1 = tmp1;
        mTmp2 = tmp2;
        mXEst = xEst;
        mHSqr = hSqr;
        mInvHSqr = invHSqr;
        mTmp3 = tmp3;
        mOmegaIe = omegaIe;
        mTmp4 = tmp4;
        mTmp5 = tmp5;
        mTmp6 = tmp6;
        mTmp7 = tmp7;
        mTmp8 = tmp8;
        mTmp9 = tmp9;
        mTmp10 = tmp10;
        mMeasurementPosition = measurementPosition;
        mMeasurementVelocity = measurementVelocity;
        mPredVelocity = predVelocity;
        mResultPosition = resultPosition;
    }

    /**
     * Constructor.
     *
     * @param measurements GNSS measurements of a collection of satellites.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final Collection<GNSSMeasurement> measurements) {
        this();
        try {
            setMeasurements(measurements);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements             GNSS measurements of a collection of satellites.
     * @param priorPositionAndVelocity previously predicted ECEF user position and
     *                                 velocity.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final Collection<GNSSMeasurement> measurements,
            final ECEFPositionAndVelocity priorPositionAndVelocity) {
        this();
        try {
            setMeasurements(measurements);
            setPriorPositionAndVelocity(priorPositionAndVelocity);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements GNSS measurement of a collection of satellites.
     * @param priorEstimation  previously predicted GNSS estimation.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final Collection<GNSSMeasurement> measurements,
            final GNSSEstimation priorEstimation) {
        this();
        try {
            setMeasurements(measurements);
            setPriorPositionAndVelocityFromEstimation(priorEstimation);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener notifying events raised by this instance.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final GNSSLeastSquaresPositionAndVelocityEstimatorListener listener) {
        this();
        try {
            setListener(listener);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements GNSS measurements of a collection of satellites.
     * @param listener     listener notifying events raised by this instance.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final Collection<GNSSMeasurement> measurements,
            final GNSSLeastSquaresPositionAndVelocityEstimatorListener listener) {
        this(measurements);
        try {
            setListener(listener);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements             GNSS measurements of a collection of satellites.
     * @param priorPositionAndVelocity previously predicted ECEF user position and
     *                                 velocity.
     * @param listener                 listener notifying events raised by this
     *                                 instance.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final Collection<GNSSMeasurement> measurements,
            final ECEFPositionAndVelocity priorPositionAndVelocity,
            final GNSSLeastSquaresPositionAndVelocityEstimatorListener listener) {
        this(measurements, priorPositionAndVelocity);
        try {
            setListener(listener);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements GNSS measurement of a collection of satellites.
     * @param priorEstimation  previously predicted GNSS estimation.
     * @param listener     listener notifying events raised by this instance.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(
            final Collection<GNSSMeasurement> measurements,
            final GNSSEstimation priorEstimation,
            final GNSSLeastSquaresPositionAndVelocityEstimatorListener listener) {
        this(measurements, priorEstimation);
        try {
            setListener(listener);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Gets GNSS measurements of a collection of satellites.
     *
     * @return GNSS measurements of a collection of satellites.
     */
    public Collection<GNSSMeasurement> getMeasurements() {
        return mMeasurements;
    }

    /**
     * Sets GNSS measurements of a collection of satellites.
     *
     * @param measurements GNSS measurements of a collection of satellites.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     * @throws LockedException          if this estimator is already running.
     */
    public void setMeasurements(
            final Collection<GNSSMeasurement> measurements) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (!isValidMeasurements(measurements)) {
            throw new IllegalArgumentException();
        }

        mMeasurements = measurements;
    }

    /**
     * Gets previously predicted ECEF user position and velocity.
     *
     * @return previously predicted ECEF user position and velocity.
     */
    public ECEFPositionAndVelocity getPriorPositionAndVelocity() {
        return mPriorPositionAndVelocity;
    }

    /**
     * Sets previously predicted ECEF user position and velocity.
     *
     * @param priorPositionAndVelocity previously predicted ECEF user position and
     *                                 velocity.
     * @throws LockedException if this estimator is already running.
     */
    public void setPriorPositionAndVelocity(
            final ECEFPositionAndVelocity priorPositionAndVelocity)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mPriorPositionAndVelocity = priorPositionAndVelocity;
    }

    /**
     * Sets previously predicted ECEF user position and velocity from a previous
     * predicted result.
     *
     * @param priorEstimation previously predicted GNSS estimation.
     * @throws LockedException if this estimator is already running.
     */
    public void setPriorPositionAndVelocityFromEstimation(
            final GNSSEstimation priorEstimation)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mPriorPositionAndVelocity = priorEstimation.getPositionAndVelocity();
    }

    /**
     * Gets listener to notify events raised by this instance.
     *
     * @return listener to notify events raised by this instance.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(
            final GNSSLeastSquaresPositionAndVelocityEstimatorListener listener)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Gets threshold to determine when convergence has been reached.
     *
     * @return threshold to determine when convergence has been reached.
     */
    public double getConvergenceThreshold() {
        return mConvergenceThreshold;
    }

    /**
     * Sets threshold to determine when convergence has been reached.
     *
     * @param convergenceThreshold threshold to determine when convergence has
     *                             been reached.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided threshold is zero or negative.
     */
    public void setConvergenceThreshold(final double convergenceThreshold)
            throws LockedException, IllegalArgumentException {
        if (mRunning) {
            throw new LockedException();
        }
        if (convergenceThreshold <= 0.0) {
            throw new IllegalArgumentException();
        }

        mConvergenceThreshold = convergenceThreshold;
    }

    /**
     * Indicates whether this estimator is ready to start the estimation.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return isValidMeasurements(mMeasurements);
    }

    /**
     * Indicates whether this estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Indicates whether provided measurements are valid or not.
     *
     * @param gnssMeasurements measurements to be checked.
     * @return true if at least 4 measurements are provided, false otherwise.
     */
    public static boolean isValidMeasurements(
            final Collection<GNSSMeasurement> gnssMeasurements) {
        return gnssMeasurements != null && gnssMeasurements.size() >= MIN_MEASUREMENTS;
    }

    /**
     * Estimates new ECEF user position and velocity as well as clock
     * offset and drift.
     *
     * @param result instance where result data will be stored.
     * @throws NotReadyException if estimator is not ready to start estimation.
     * @throws LockedException   if estimator is already running.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    public void estimate(final GNSSEstimation result)
            throws NotReadyException, LockedException, GNSSException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (mRunning) {
            throw new LockedException();
        }

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            // if no prior position and velocity is available, assume that
            // we are at latitude,longitude equal to the average of satellite
            // measurements, at Earth's surface (height = 0) and with zero velocity.
            initializePriorPositionAndVelocityIfNeeded();

            // POSITION AND CLOCK OFFSET

            // Setup predicted state
            final double priorX = mPriorPositionAndVelocity.getX();
            final double priorY = mPriorPositionAndVelocity.getY();
            final double priorZ = mPriorPositionAndVelocity.getZ();

            mXPred.setElementAtIndex(0, priorX);
            mXPred.setElementAtIndex(1, priorY);
            mXPred.setElementAtIndex(2, priorZ);
            mXPred.setElementAtIndex(3, 0.0);

            final int numMeasurements = mMeasurements.size();
            final Matrix predMeas = new Matrix(numMeasurements, 1);
            final Matrix h = new Matrix(numMeasurements, STATE_COMPONENTS);
            for (int i = 0; i < numMeasurements; i++) {
                h.setElementAt(i, 3, 1.0);
            }

            final Matrix hTrans = new Matrix(STATE_COMPONENTS, numMeasurements);
            final Matrix hTmp1 = new Matrix(STATE_COMPONENTS, numMeasurements);
            final Matrix deltaPseudoRange = new Matrix(numMeasurements, 1);

            // Repeat until convergence
            double testConvergence = 1.0;
            while (testConvergence > mConvergenceThreshold) {

                // Loop measurements
                int j = 0;
                for (final GNSSMeasurement measurement : mMeasurements) {

                    // Predict approx range
                    final double measX = measurement.getX();
                    final double measY = measurement.getY();
                    final double measZ = measurement.getZ();

                    double deltaRx = measX - priorX;
                    double deltaRy = measY - priorY;
                    double deltaRz = measZ - priorZ;
                    final double approxRange = norm(deltaRx, deltaRy, deltaRz);

                    // Calculate frame rotation during signal transit time using (8.36)
                    final double ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
                    mCei.setElementAt(0, 1, ceiValue);
                    mCei.setElementAt(1, 0, -ceiValue);

                    // Predict pseudo-range using (9.143)
                    mTmp1.setElementAtIndex(0, measX);
                    mTmp1.setElementAtIndex(1, measY);
                    mTmp1.setElementAtIndex(2, measZ);

                    mCei.multiply(mTmp1, mTmp2);

                    deltaRx = mTmp2.getElementAtIndex(0) - mXPred.getElementAtIndex(0);
                    deltaRy = mTmp2.getElementAtIndex(1) - mXPred.getElementAtIndex(1);
                    deltaRz = mTmp2.getElementAtIndex(2) - mXPred.getElementAtIndex(2);
                    final double range = norm(deltaRx, deltaRy, deltaRz);

                    final double predictedPseudoRange = range + mXPred.getElementAtIndex(3);
                    predMeas.setElementAtIndex(j, predictedPseudoRange);

                    deltaPseudoRange.setElementAtIndex(j,
                            measurement.getPseudoRange() - predictedPseudoRange);

                    // Predict line of sight and deploy in measurement matrix, (9.144)
                    h.setElementAt(j, 0, -deltaRx / range);
                    h.setElementAt(j, 1, -deltaRy / range);
                    h.setElementAt(j, 2, -deltaRz / range);

                    j++;
                }

                // Unweighted least-squares solution, (9.35)/(9.141)
                h.transpose(hTrans);
                hTrans.multiply(h, mHSqr);
                Utils.inverse(mHSqr, mInvHSqr);
                mInvHSqr.multiply(hTrans, hTmp1);
                hTmp1.multiply(deltaPseudoRange, mTmp3);

                mXPred.add(mTmp3, mXEst);

                // Test convergence
                testConvergence = predictionError();

                // Set predictions to estimates for next iteration
                mXPred.copyFrom(mXEst);
            }

            // Set outputs to estimates
            final double resultX = mXEst.getElementAtIndex(0);
            final double resultY = mXEst.getElementAtIndex(1);
            final double resultZ = mXEst.getElementAtIndex(2);
            result.setPositionCoordinates(resultX, resultY, resultZ);

            final double resultClockOffset = mXEst.getElementAtIndex(3);
            result.setClockOffset(resultClockOffset);


            // VELOCITY AND CLOCK DRIFT

            // Setup predicted state
            final double priorVx = mPriorPositionAndVelocity.getVx();
            final double priorVy = mPriorPositionAndVelocity.getVy();
            final double priorVz = mPriorPositionAndVelocity.getVz();

            mXPred.setElementAtIndex(0, priorVx);
            mXPred.setElementAtIndex(1, priorVy);
            mXPred.setElementAtIndex(2, priorVz);
            mXPred.setElementAtIndex(3, 0.0);

            mResultPosition.setElementAtIndex(0, resultX);
            mResultPosition.setElementAtIndex(1, resultY);
            mResultPosition.setElementAtIndex(2, resultZ);

            final Matrix deltaPseudoRangeRate = new Matrix(numMeasurements, 1);

            // Repeat until convergence
            testConvergence = 1.0;
            while (testConvergence > mConvergenceThreshold) {

                // Loop measurements
                int j = 0;
                for (final GNSSMeasurement measurement : mMeasurements) {
                    // Predict approx range
                    final double measX = measurement.getX();
                    final double measY = measurement.getY();
                    final double measZ = measurement.getZ();

                    double deltaRx = measX - resultX;
                    double deltaRy = measY - resultY;
                    double deltaRz = measZ - resultZ;
                    final double approxRange = norm(deltaRx, deltaRy, deltaRz);

                    // Calculate frame rotation during signal transit time using (8.36)
                    final double ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
                    mCei.setElementAt(0, 1, ceiValue);
                    mCei.setElementAt(1, 0, -ceiValue);

                    // Calculate range using (8.35)
                    mTmp1.setElementAtIndex(0, measX);
                    mTmp1.setElementAtIndex(1, measY);
                    mTmp1.setElementAtIndex(2, measZ);

                    mCei.multiply(mTmp1, mTmp2);

                    deltaRx = mTmp2.getElementAtIndex(0) - resultX;
                    deltaRy = mTmp2.getElementAtIndex(1) - resultY;
                    deltaRz = mTmp2.getElementAtIndex(2) - resultZ;
                    final double range = norm(deltaRx, deltaRy, deltaRz);

                    // Calculate line of sight using (8.41)
                    final double uaseX = deltaRx / range;
                    final double uaseY = deltaRy / range;
                    final double uaseZ = deltaRz / range;

                    // Predict pseudo-range rate using (9.143)
                    mMeasurementPosition.setElementAtIndex(0, measX);
                    mMeasurementPosition.setElementAtIndex(1, measY);
                    mMeasurementPosition.setElementAtIndex(2, measZ);

                    final double measVx = measurement.getVx();
                    final double measVy = measurement.getVy();
                    final double measVz = measurement.getVz();

                    mMeasurementVelocity.setElementAtIndex(0, measVx);
                    mMeasurementVelocity.setElementAtIndex(1, measVy);
                    mMeasurementVelocity.setElementAtIndex(2, measVz);

                    mOmegaIe.multiply(mMeasurementPosition, mTmp4);

                    mMeasurementVelocity.add(mTmp4, mTmp5);

                    mCei.multiply(mTmp5, mTmp6);

                    mOmegaIe.multiply(mResultPosition, mTmp7);

                    mXPred.getSubmatrix(0, 0,
                            ELEMS_MINUS_ONE, 0, mPredVelocity);

                    mPredVelocity.add(mTmp7, mTmp8);

                    mTmp6.subtract(mTmp8, mTmp9);

                    final double rangeRate = uaseX * mTmp9.getElementAtIndex(0)
                            + uaseY * mTmp9.getElementAtIndex(1)
                            + uaseZ * mTmp9.getElementAtIndex(2);

                    final double predictedPseudoRangeRate = rangeRate + mXPred.getElementAtIndex(3);
                    predMeas.setElementAtIndex(j, predictedPseudoRangeRate);

                    deltaPseudoRangeRate.setElementAtIndex(j,
                            measurement.getPseudoRate() - predictedPseudoRangeRate);

                    // Predict line of sight and deploy in measurement matrix, (9.144)
                    h.setElementAt(j, 0, -uaseX);
                    h.setElementAt(j, 1, -uaseY);
                    h.setElementAt(j, 2, -uaseZ);

                    j++;
                }

                // Unweighted least-squares solution, (9.35)/(9.141)
                h.transpose(hTrans);
                hTrans.multiply(h, mHSqr);
                Utils.inverse(mHSqr, mInvHSqr);
                mInvHSqr.multiply(hTrans, hTmp1);
                hTmp1.multiply(deltaPseudoRangeRate, mTmp10);

                mXPred.add(mTmp10, mXEst);

                // Test convergence
                testConvergence = predictionError();

                // Set predictions to estimates for next iteration
                mXPred.copyFrom(mXEst);
            }

            // Set outputs to estimates
            final double resultVx = mXEst.getElementAtIndex(0);
            final double resultVy = mXEst.getElementAtIndex(1);
            final double resultVz = mXEst.getElementAtIndex(2);
            result.setVelocityCoordinates(resultVx, resultVy, resultVz);

            final double resultClockDrift = mXEst.getElementAtIndex(3);
            result.setClockDrift(resultClockDrift);

        } catch (final AlgebraException e) {
            throw new GNSSException(e);
        } finally {

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }

            mRunning = false;
        }
    }

    /**
     * Estimates new ECEF user position and velocity as well as clock
     * offset and drift.
     *
     * @return new ECEF user position and velocity, and clock offset and drift.
     * @throws NotReadyException if estimator is not ready to start estimation.
     * @throws LockedException   if estimator is already running.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    public GNSSEstimation estimate()
            throws NotReadyException, LockedException, GNSSException {
        final GNSSEstimation result =
                new GNSSEstimation();
        estimate(result);
        return result;
    }

    /**
     * Initializes prior position and velocity if not set, assuming that
     * user is located at the average latitude,longitud of all provided
     * satellite measurements, at Earth's surface (height = 0) and with zero velocity.
     */
    private void initializePriorPositionAndVelocityIfNeeded() {
        if (mPriorPositionAndVelocity != null) {
            return;
        }

        int numMeasurements = mMeasurements.size();
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();

        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();

        double userLatitude = 0.0;
        double userLongitude = 0.0;
        for (final GNSSMeasurement measurement : mMeasurements) {
            measurement.getEcefPosition(ecefPosition);
            measurement.getEcefVelocity(ecefVelocity);
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                    ecefPosition, ecefVelocity, nedPosition, nedVelocity);

            final double satLatitude = nedPosition.getLatitude();
            final double satLongitude = nedPosition.getLongitude();

            userLatitude += satLatitude / numMeasurements;
            userLongitude += satLongitude / numMeasurements;
        }

        nedPosition.setCoordinates(userLatitude, userLongitude, 0.0);
        nedVelocity.setCoordinates(0.0, 0.0, 0.0);

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        mPriorPositionAndVelocity = new ECEFPositionAndVelocity(
                ecefPosition, ecefVelocity);
    }

    /**
     * Computes norm of provided coordinates.
     *
     * @param x x coordinate.
     * @param y y coordinate.
     * @param z z coordinate.
     * @return computed norm.
     */
    private static double norm(final double x, final double y, final double z) {
        return Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * Computes norm of error between estimated state
     * and predicted state.
     *
     * @return norm of error.
     */
    private double predictionError() {
        double sqrPredictionError = 0.0;
        for (int i = 0; i < STATE_COMPONENTS; i++) {
            final double diff = mXEst.getElementAtIndex(i)
                    - mXPred.getElementAtIndex(i);
            sqrPredictionError += diff * diff;
        }
        return Math.sqrt(sqrPredictionError);
    }
}
