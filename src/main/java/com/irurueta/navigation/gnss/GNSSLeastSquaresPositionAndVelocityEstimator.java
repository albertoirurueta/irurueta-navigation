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
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;

import java.util.Collection;

/**
 * Calculates position, velocity, clock offset and clock drift using
 * unweighted iterated least squares.
 * Separate calculations are implemented for position and clock offset and
 * for velocity and clock drift.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_LS_position_velocity.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_LS_position_velocity.m
 * </a>
 */
public class GNSSLeastSquaresPositionAndVelocityEstimator {

    /**
     * Minimum number of measurements required to obtain a solution.
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
    private Collection<GNSSMeasurement> measurements;

    /**
     * Previously predicted ECEF user position and velocity.
     */
    private ECEFPositionAndVelocity priorPositionAndVelocity;

    /**
     * Listener to notify events raised by this instance.
     */
    private GNSSLeastSquaresPositionAndVelocityEstimatorListener listener;

    /**
     * Threshold to determine when convergence has been reached.
     */
    private double convergenceThreshold = CONVERGENCE_THRESHOLD;

    /**
     * Indicates whether estimation is currently running.
     */
    private boolean running;

    /**
     * Internal matrix to be reused containing frame rotation during signal transit
     * time.
     */
    private final Matrix cei;

    /**
     * Predicted state to be reused.
     */
    private final Matrix xPred;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp1;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp2;

    /**
     * Estimated state to be reused.
     */
    private final Matrix xEst;

    /**
     * Contains square representation of measurement or geometry matrix.
     */
    private final Matrix hSqr;

    /**
     * Inverse of the square representation of measurement or geometry matrix.
     */
    private final Matrix invHSqr;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp3;

    /**
     * Skew symmetric matrix of Earth rotation rate.
     */
    private final Matrix omegaIe;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp4;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp5;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp6;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp7;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp8;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp9;

    /**
     * Temporal matrix to be reused.
     */
    private final Matrix tmp10;

    /**
     * Measurement position to be reused.
     */
    private final Matrix measurementPosition;

    /**
     * Measurement velocity to be reused.
     */
    private final Matrix measurementVelocity;

    /**
     * Predicted velocity to be reused.
     */
    private final Matrix predVelocity;

    /**
     * Result position to be reused.
     */
    private final Matrix resultPosition;

    /**
     * Constructor.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator() {
        Matrix cxPred = null;
        Matrix ctmp1 = null;
        Matrix ctmp2 = null;
        Matrix cxEst = null;
        Matrix chSqr = null;
        Matrix cinvHSqr = null;
        Matrix ctmp3 = null;
        Matrix comegaIe = null;
        Matrix ccei = null;
        Matrix ctmp4 = null;
        Matrix ctmp5 = null;
        Matrix ctmp6 = null;
        Matrix ctmp7 = null;
        Matrix ctmp8 = null;
        Matrix ctmp9 = null;
        Matrix ctmp10 = null;
        Matrix cmeasurementPosition = null;
        Matrix cmeasurementVelocity = null;
        Matrix cpredVelocity = null;
        Matrix cresultPosition = null;
        try {
            ccei = Matrix.identity(ELEMS, ELEMS);
            cxPred = new Matrix(STATE_COMPONENTS, 1);
            ctmp1 = new Matrix(ELEMS, 1);
            ctmp2 = new Matrix(ELEMS, 1);
            cxEst = new Matrix(STATE_COMPONENTS, 1);
            chSqr = new Matrix(STATE_COMPONENTS, STATE_COMPONENTS);
            cinvHSqr = new Matrix(STATE_COMPONENTS, STATE_COMPONENTS);
            ctmp3 = new Matrix(STATE_COMPONENTS, 1);
            comegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, EARTH_ROTATION_RATE});
            ctmp4 = new Matrix(STATE_COMPONENTS, 1);
            ctmp5 = new Matrix(STATE_COMPONENTS, 1);
            ctmp6 = new Matrix(STATE_COMPONENTS, 1);
            ctmp7 = new Matrix(STATE_COMPONENTS, 1);
            ctmp8 = new Matrix(STATE_COMPONENTS, 1);
            ctmp9 = new Matrix(STATE_COMPONENTS, 1);
            ctmp10 = new Matrix(STATE_COMPONENTS, 1);
            cmeasurementPosition = new Matrix(ELEMS, 1);
            cmeasurementVelocity = new Matrix(ELEMS, 1);
            cpredVelocity = new Matrix(ELEMS, 1);
            cresultPosition = new Matrix(ELEMS, 1);
        } catch (WrongSizeException ignore) {
            // never happens
        }

        this.cei = ccei;
        this.xPred = cxPred;
        this.tmp1 = ctmp1;
        this.tmp2 = ctmp2;
        this.xEst = cxEst;
        this.hSqr = chSqr;
        this.invHSqr = cinvHSqr;
        this.tmp3 = ctmp3;
        this.omegaIe = comegaIe;
        this.tmp4 = ctmp4;
        this.tmp5 = ctmp5;
        this.tmp6 = ctmp6;
        this.tmp7 = ctmp7;
        this.tmp8 = ctmp8;
        this.tmp9 = ctmp9;
        this.tmp10 = ctmp10;
        this.measurementPosition = cmeasurementPosition;
        this.measurementVelocity = cmeasurementVelocity;
        this.predVelocity = cpredVelocity;
        this.resultPosition = cresultPosition;
    }

    /**
     * Constructor.
     *
     * @param measurements GNSS measurements of a collection of satellites.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimator(final Collection<GNSSMeasurement> measurements) {
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
        return measurements;
    }

    /**
     * Sets GNSS measurements of a collection of satellites.
     *
     * @param measurements GNSS measurements of a collection of satellites.
     * @throws IllegalArgumentException if less than 4 measurements are provided.
     * @throws LockedException          if this estimator is already running.
     */
    public void setMeasurements(final Collection<GNSSMeasurement> measurements) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (!isValidMeasurements(measurements)) {
            throw new IllegalArgumentException();
        }

        this.measurements = measurements;
    }

    /**
     * Gets previously predicted ECEF user position and velocity.
     *
     * @return previously predicted ECEF user position and velocity.
     */
    public ECEFPositionAndVelocity getPriorPositionAndVelocity() {
        return priorPositionAndVelocity;
    }

    /**
     * Sets previously predicted ECEF user position and velocity.
     *
     * @param priorPositionAndVelocity previously predicted ECEF user position and
     *                                 velocity.
     * @throws LockedException if this estimator is already running.
     */
    public void setPriorPositionAndVelocity(
            final ECEFPositionAndVelocity priorPositionAndVelocity) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.priorPositionAndVelocity = priorPositionAndVelocity;
    }

    /**
     * Sets previously predicted ECEF user position and velocity from a previous
     * predicted result.
     *
     * @param priorEstimation previously predicted GNSS estimation.
     * @throws LockedException if this estimator is already running.
     */
    public void setPriorPositionAndVelocityFromEstimation(
            final GNSSEstimation priorEstimation) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        priorPositionAndVelocity = priorEstimation != null ? priorEstimation.getPositionAndVelocity() : null;
    }

    /**
     * Gets listener to notify events raised by this instance.
     *
     * @return listener to notify events raised by this instance.
     */
    public GNSSLeastSquaresPositionAndVelocityEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(
            final GNSSLeastSquaresPositionAndVelocityEstimatorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Gets threshold to determine when convergence has been reached.
     *
     * @return threshold to determine when convergence has been reached.
     */
    public double getConvergenceThreshold() {
        return convergenceThreshold;
    }

    /**
     * Sets threshold to determine when convergence has been reached.
     *
     * @param convergenceThreshold threshold to determine when convergence has
     *                             been reached.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided threshold is zero or negative.
     */
    public void setConvergenceThreshold(final double convergenceThreshold) throws LockedException,
            IllegalArgumentException {
        if (running) {
            throw new LockedException();
        }
        if (convergenceThreshold <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.convergenceThreshold = convergenceThreshold;
    }

    /**
     * Indicates whether this estimator is ready to start the estimation.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return isValidMeasurements(measurements);
    }

    /**
     * Indicates whether this estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Indicates whether provided measurements are valid or not.
     *
     * @param gnssMeasurements measurements to be checked.
     * @return true if at least 4 measurements are provided, false otherwise.
     */
    public static boolean isValidMeasurements(final Collection<GNSSMeasurement> gnssMeasurements) {
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
    @SuppressWarnings("DuplicatedCode")
    public void estimate(final GNSSEstimation result) throws NotReadyException, LockedException, GNSSException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (running) {
            throw new LockedException();
        }

        try {
            running = true;

            if (listener != null) {
                listener.onEstimateStart(this);
            }

            // if no prior position and velocity is available, assume that
            // we are at latitude,longitude equal to the average of satellite
            // measurements, at Earth's surface (height = 0) and with zero velocity.
            initializePriorPositionAndVelocityIfNeeded();

            // POSITION AND CLOCK OFFSET

            // Setup predicted state
            final var priorX = priorPositionAndVelocity.getX();
            final var priorY = priorPositionAndVelocity.getY();
            final var priorZ = priorPositionAndVelocity.getZ();

            xPred.setElementAtIndex(0, priorX);
            xPred.setElementAtIndex(1, priorY);
            xPred.setElementAtIndex(2, priorZ);
            xPred.setElementAtIndex(3, 0.0);

            final var numMeasurements = measurements.size();
            final var predMeas = new Matrix(numMeasurements, 1);
            final var h = new Matrix(numMeasurements, STATE_COMPONENTS);
            for (var i = 0; i < numMeasurements; i++) {
                h.setElementAt(i, 3, 1.0);
            }

            final var hTrans = new Matrix(STATE_COMPONENTS, numMeasurements);
            final var hTmp1 = new Matrix(STATE_COMPONENTS, numMeasurements);
            final var deltaPseudoRange = new Matrix(numMeasurements, 1);

            // Repeat until convergence
            var testConvergence = 1.0;
            while (testConvergence > convergenceThreshold) {

                // Loop measurements
                var j = 0;
                for (final var measurement : measurements) {

                    // Predict approx range
                    final var measX = measurement.getX();
                    final var measY = measurement.getY();
                    final var measZ = measurement.getZ();

                    var deltaRx = measX - priorX;
                    var deltaRy = measY - priorY;
                    var deltaRz = measZ - priorZ;
                    final var approxRange = norm(deltaRx, deltaRy, deltaRz);

                    // Calculate frame rotation during signal transit time using (8.36)
                    final var ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
                    cei.setElementAt(0, 1, ceiValue);
                    cei.setElementAt(1, 0, -ceiValue);

                    // Predict pseudo-range using (9.143)
                    tmp1.setElementAtIndex(0, measX);
                    tmp1.setElementAtIndex(1, measY);
                    tmp1.setElementAtIndex(2, measZ);

                    cei.multiply(tmp1, tmp2);

                    deltaRx = tmp2.getElementAtIndex(0) - xPred.getElementAtIndex(0);
                    deltaRy = tmp2.getElementAtIndex(1) - xPred.getElementAtIndex(1);
                    deltaRz = tmp2.getElementAtIndex(2) - xPred.getElementAtIndex(2);
                    final var range = norm(deltaRx, deltaRy, deltaRz);

                    final var predictedPseudoRange = range + xPred.getElementAtIndex(3);
                    predMeas.setElementAtIndex(j, predictedPseudoRange);

                    deltaPseudoRange.setElementAtIndex(j, measurement.getPseudoRange() - predictedPseudoRange);

                    // Predict line of sight and deploy in measurement matrix, (9.144)
                    h.setElementAt(j, 0, -deltaRx / range);
                    h.setElementAt(j, 1, -deltaRy / range);
                    h.setElementAt(j, 2, -deltaRz / range);

                    j++;
                }

                // Unweighted least-squares solution, (9.35)/(9.141)
                h.transpose(hTrans);
                hTrans.multiply(h, hSqr);
                Utils.inverse(hSqr, invHSqr);
                invHSqr.multiply(hTrans, hTmp1);
                hTmp1.multiply(deltaPseudoRange, tmp3);

                xPred.add(tmp3, xEst);

                // Test convergence
                testConvergence = predictionError();

                // Set predictions to estimates for next iteration
                xPred.copyFrom(xEst);
            }

            // Set outputs to estimates
            final var resultX = xEst.getElementAtIndex(0);
            final var resultY = xEst.getElementAtIndex(1);
            final var resultZ = xEst.getElementAtIndex(2);
            result.setPositionCoordinates(resultX, resultY, resultZ);

            final var resultClockOffset = xEst.getElementAtIndex(3);
            result.setClockOffset(resultClockOffset);


            // VELOCITY AND CLOCK DRIFT

            // Setup predicted state
            final var priorVx = priorPositionAndVelocity.getVx();
            final var priorVy = priorPositionAndVelocity.getVy();
            final var priorVz = priorPositionAndVelocity.getVz();

            xPred.setElementAtIndex(0, priorVx);
            xPred.setElementAtIndex(1, priorVy);
            xPred.setElementAtIndex(2, priorVz);
            xPred.setElementAtIndex(3, 0.0);

            resultPosition.setElementAtIndex(0, resultX);
            resultPosition.setElementAtIndex(1, resultY);
            resultPosition.setElementAtIndex(2, resultZ);

            final var deltaPseudoRangeRate = new Matrix(numMeasurements, 1);

            // Repeat until convergence
            testConvergence = 1.0;
            while (testConvergence > convergenceThreshold) {

                // Loop measurements
                var j = 0;
                for (final var measurement : measurements) {
                    // Predict approx range
                    final var measX = measurement.getX();
                    final var measY = measurement.getY();
                    final var measZ = measurement.getZ();

                    var deltaRx = measX - resultX;
                    var deltaRy = measY - resultY;
                    var deltaRz = measZ - resultZ;
                    final var approxRange = norm(deltaRx, deltaRy, deltaRz);

                    // Calculate frame rotation during signal transit time using (8.36)
                    final var ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
                    cei.setElementAt(0, 1, ceiValue);
                    cei.setElementAt(1, 0, -ceiValue);

                    // Calculate range using (8.35)
                    tmp1.setElementAtIndex(0, measX);
                    tmp1.setElementAtIndex(1, measY);
                    tmp1.setElementAtIndex(2, measZ);

                    cei.multiply(tmp1, tmp2);

                    deltaRx = tmp2.getElementAtIndex(0) - resultX;
                    deltaRy = tmp2.getElementAtIndex(1) - resultY;
                    deltaRz = tmp2.getElementAtIndex(2) - resultZ;
                    final var range = norm(deltaRx, deltaRy, deltaRz);

                    // Calculate line of sight using (8.41)
                    final var uaseX = deltaRx / range;
                    final var uaseY = deltaRy / range;
                    final var uaseZ = deltaRz / range;

                    // Predict pseudo-range rate using (9.143)
                    measurementPosition.setElementAtIndex(0, measX);
                    measurementPosition.setElementAtIndex(1, measY);
                    measurementPosition.setElementAtIndex(2, measZ);

                    final var measVx = measurement.getVx();
                    final var measVy = measurement.getVy();
                    final var measVz = measurement.getVz();

                    measurementVelocity.setElementAtIndex(0, measVx);
                    measurementVelocity.setElementAtIndex(1, measVy);
                    measurementVelocity.setElementAtIndex(2, measVz);

                    omegaIe.multiply(measurementPosition, tmp4);

                    measurementVelocity.add(tmp4, tmp5);

                    cei.multiply(tmp5, tmp6);

                    omegaIe.multiply(resultPosition, tmp7);

                    xPred.getSubmatrix(0, 0, ELEMS_MINUS_ONE, 0, predVelocity);

                    predVelocity.add(tmp7, tmp8);

                    tmp6.subtract(tmp8, tmp9);

                    final var rangeRate = uaseX * tmp9.getElementAtIndex(0) + uaseY * tmp9.getElementAtIndex(1)
                            + uaseZ * tmp9.getElementAtIndex(2);

                    final var predictedPseudoRangeRate = rangeRate + xPred.getElementAtIndex(3);
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
                hTrans.multiply(h, hSqr);
                Utils.inverse(hSqr, invHSqr);
                invHSqr.multiply(hTrans, hTmp1);
                hTmp1.multiply(deltaPseudoRangeRate, tmp10);

                xPred.add(tmp10, xEst);

                // Test convergence
                testConvergence = predictionError();

                // Set predictions to estimates for next iteration
                xPred.copyFrom(xEst);
            }

            // Set outputs to estimates
            final var resultVx = xEst.getElementAtIndex(0);
            final var resultVy = xEst.getElementAtIndex(1);
            final var resultVz = xEst.getElementAtIndex(2);
            result.setVelocityCoordinates(resultVx, resultVy, resultVz);

            final var resultClockDrift = xEst.getElementAtIndex(3);
            result.setClockDrift(resultClockDrift);

        } catch (final AlgebraException e) {
            throw new GNSSException(e);
        } finally {
            if (listener != null) {
                listener.onEstimateEnd(this);
            }

            running = false;
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
    public GNSSEstimation estimate() throws NotReadyException, LockedException, GNSSException {
        final var result = new GNSSEstimation();
        estimate(result);
        return result;
    }

    /**
     * Initializes prior position and velocity if not set, assuming that
     * user is located at the average latitude, longitude of all provided
     * satellite measurements, at Earth's surface (height = 0) and with zero velocity.
     */
    private void initializePriorPositionAndVelocityIfNeeded() {
        if (priorPositionAndVelocity != null) {
            return;
        }

        var numMeasurements = measurements.size();
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();

        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();

        var userLatitude = 0.0;
        var userLongitude = 0.0;
        for (final var measurement : measurements) {
            measurement.getEcefPosition(ecefPosition);
            measurement.getEcefVelocity(ecefVelocity);
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(ecefPosition, ecefVelocity, nedPosition, nedVelocity);

            final var satLatitude = nedPosition.getLatitude();
            final var satLongitude = nedPosition.getLongitude();

            userLatitude += satLatitude / numMeasurements;
            userLongitude += satLongitude / numMeasurements;
        }

        nedPosition.setCoordinates(userLatitude, userLongitude, 0.0);
        nedVelocity.setCoordinates(0.0, 0.0, 0.0);

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        priorPositionAndVelocity = new ECEFPositionAndVelocity(ecefPosition, ecefVelocity);
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
        var sqrPredictionError = 0.0;
        for (var i = 0; i < STATE_COMPONENTS; i++) {
            final var diff = xEst.getElementAtIndex(i) - xPred.getElementAtIndex(i);
            sqrPredictionError += diff * diff;
        }
        return Math.sqrt(sqrPredictionError);
    }
}
