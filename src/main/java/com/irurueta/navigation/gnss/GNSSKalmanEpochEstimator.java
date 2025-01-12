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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.Collection;


/**
 * Implements one cycle of the GNSS extended Kalman filter.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_KF_Epoch.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/GNSS_KF_Epoch.m
 * </a>
 */
public class GNSSKalmanEpochEstimator {

    /**
     * Speed of light in the vacuum expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = Constants.SPEED_OF_LIGHT;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Number of rows and columns of transition and system noise covariance matrices.
     */
    private static final int MATRIX_SIZE = 8;

    /**
     * Constructor.
     * Prevents instantiation of utility class.
     */
    private GNSSKalmanEpochEstimator() {
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous GNSS estimates and Kalman filter error
     *                            covariance matrix.
     * @param config              system configuration (usually obtained through
     *                            calibration).
     * @return new Kalman filter state.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static GNSSKalmanState estimate(
            final Collection<GNSSMeasurement> measurements, final Time propagationInterval,
            final GNSSKalmanState previousState, final GNSSKalmanConfig config) throws AlgebraException {
        return estimate(measurements, convertTime(propagationInterval), previousState, config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix fo a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previousGNSS estimates and Kalman filter error
     *                            covariance matrix.
     * @param config              system configuration (usually obtained through
     *                            calibration).
     * @param result              instance where updated Kalman filter state will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(final Collection<GNSSMeasurement> measurements,
                                final Time propagationInterval,
                                final GNSSKalmanState previousState,
                                final GNSSKalmanConfig config,
                                final GNSSKalmanState result) throws AlgebraException {
        estimate(measurements, convertTime(propagationInterval), previousState, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousEstimation  previousGNSS estimates.
     * @param previousCovariance  previousKalman filter error covariance matrix.
     * @param config              system configuration (usually obtained through
     *                            calibration).
     * @param updatedEstimation   instance where updated GNSS estimate will be stored
     *                            after executing this method.
     * @param updatedCovariance   instance where updated Kalman filter error covariance
     *                            matrix will be stored.
     * @throws IllegalArgumentException if provided previous covariance matrix is not
     *                                  8x8.
     * @throws AlgebraException         if there are numerical instabilities.
     */
    public static void estimate(final Collection<GNSSMeasurement> measurements,
                                final Time propagationInterval,
                                final GNSSEstimation previousEstimation,
                                final Matrix previousCovariance,
                                final GNSSKalmanConfig config,
                                final GNSSEstimation updatedEstimation,
                                final Matrix updatedCovariance) throws AlgebraException {
        estimate(measurements, convertTime(propagationInterval), previousEstimation, previousCovariance, config,
                updatedEstimation, updatedCovariance);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous GNSS estimates and Kalman filter error
     *                            covariance matrix.
     * @param config              system configuration (usually obtained through
     *                            calibration).
     * @return new Kalman filter state.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static GNSSKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final GNSSKalmanState previousState,
            final GNSSKalmanConfig config) throws AlgebraException {
        final var result = new GNSSKalmanState();
        estimate(measurements, propagationInterval, previousState, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous GNSS estimates and Kalman filter error
     *                            covariance matrix.
     * @param config              system configuration (usually obtained through
     *                            calibration).
     * @param result              instance where updated Kalman filter state will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(final Collection<GNSSMeasurement> measurements,
                                final double propagationInterval,
                                final GNSSKalmanState previousState,
                                final GNSSKalmanConfig config,
                                final GNSSKalmanState result) throws AlgebraException {
        final var resultEstimation = new GNSSEstimation();
        final var resultCovariance = new Matrix(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);

        estimate(measurements, propagationInterval, previousState.getEstimation(), previousState.getCovariance(),
                config, resultEstimation, resultCovariance);

        result.setEstimation(resultEstimation);
        result.setCovariance(resultCovariance);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousEstimation  previous GNSS estimates.
     * @param previousCovariance  previous Kalman filter error covariance matrix.
     * @param config              system configuration (usually obtained through
     *                            calibration).
     * @param updatedEstimation   instance where updated GNSS estimate will be stored
     *                            after executing this method.
     * @param updatedCovariance   instance where updated Kalman filter error covariance
     *                            matrix will be stored.
     * @throws IllegalArgumentException if provided previous covariance matrix is not
     *                                  8x8.
     * @throws AlgebraException         if there are numerical instabilities.
     */
    @SuppressWarnings("DuplicatedCode")
    public static void estimate(final Collection<GNSSMeasurement> measurements,
                                final double propagationInterval,
                                final GNSSEstimation previousEstimation,
                                final Matrix previousCovariance,
                                final GNSSKalmanConfig config,
                                final GNSSEstimation updatedEstimation,
                                final Matrix updatedCovariance) throws AlgebraException {

        if (previousCovariance.getRows() != GNSSEstimation.NUM_PARAMETERS
                || previousCovariance.getColumns() != GNSSEstimation.NUM_PARAMETERS) {
            throw new IllegalArgumentException();
        }

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (9.147) and (9.150)
        final var phiMatrix = Matrix.identity(MATRIX_SIZE, MATRIX_SIZE);
        phiMatrix.setElementAt(0, 3, propagationInterval);
        phiMatrix.setElementAt(1, 4, propagationInterval);
        phiMatrix.setElementAt(2, 5, propagationInterval);
        phiMatrix.setElementAt(6, 7, propagationInterval);

        // 2. Determine system noise covariance matrix using (9.152)
        final var propagationInterval2 = propagationInterval * propagationInterval;
        final var propagationInterval3 = propagationInterval2 * propagationInterval;
        final var accelerationPSD = config.getAccelerationPSD();
        final var clockFrequencyPSD = config.getClockFrequencyPSD();
        final var clockPhasePSD = config.getClockPhasePSD();

        final var value1 = accelerationPSD * propagationInterval3 / 3.0;
        final var value2 = accelerationPSD * propagationInterval2 / 2.0;
        final var value3 = accelerationPSD * propagationInterval;
        final var value4 = clockFrequencyPSD * propagationInterval3 / 3.0 + clockPhasePSD * propagationInterval;
        final var value5 = clockFrequencyPSD * propagationInterval2 / 2.0;
        final var value6 = clockFrequencyPSD * propagationInterval;

        final var qMatrix = new Matrix(MATRIX_SIZE, MATRIX_SIZE);
        qMatrix.setElementAt(0, 0, value1);
        qMatrix.setElementAt(1, 1, value1);
        qMatrix.setElementAt(2, 2, value1);

        qMatrix.setElementAt(0, 3, value2);
        qMatrix.setElementAt(1, 4, value2);
        qMatrix.setElementAt(2, 5, value2);

        qMatrix.setElementAt(3, 0, value2);
        qMatrix.setElementAt(4, 1, value2);
        qMatrix.setElementAt(5, 2, value2);

        qMatrix.setElementAt(3, 3, value3);
        qMatrix.setElementAt(4, 4, value3);
        qMatrix.setElementAt(5, 5, value3);

        qMatrix.setElementAt(6, 6, value4);
        qMatrix.setElementAt(6, 7, value5);
        qMatrix.setElementAt(7, 6, value5);
        qMatrix.setElementAt(7, 7, value6);


        // 3. Propagate state estimates using (3.14)
        final var xEstOld = previousEstimation.asMatrix();
        final var xEstPropagated = phiMatrix.multiplyAndReturnNew(xEstOld);
        final var propagatedVelocity = xEstPropagated.getSubmatrix(
                3, 0, 5, 0);
        final var propagatedPosition = xEstPropagated.getSubmatrix(
                0, 0, 2, 0);

        // 4. Propagate state estimation error covariance matrix using (3.15)
        final var pMatrixPropagated = phiMatrix.multiplyAndReturnNew(previousCovariance);
        phiMatrix.transpose();
        pMatrixPropagated.multiply(phiMatrix);
        pMatrixPropagated.add(qMatrix);

        // MEASUREMENT UPDATE PHASE

        // Skew symmetric matrix of Earth rate
        final var omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, EARTH_ROTATION_RATE});

        final var numberOfMeasurements = measurements.size();
        final var uAseT = new Matrix(numberOfMeasurements, 3);
        final var predMeas = new Matrix(numberOfMeasurements, 2);

        final var cei = Matrix.identity(CoordinateTransformation.ROWS, CoordinateTransformation.COLS);
        final var satellitePosition = new Matrix(CoordinateTransformation.ROWS, 1);
        final var satelliteVelocity = new Matrix(CoordinateTransformation.ROWS, 1);
        final var deltaR = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp1 = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp2 = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp3 = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp4 = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp5 = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp6 = new Matrix(CoordinateTransformation.ROWS, 1);
        final var tmp7 = new Matrix(1, CoordinateTransformation.ROWS);

        // Loop measurements
        var j = 0;
        for (final var measurement : measurements) {
            // Predict approx range
            final var measX = measurement.getX();
            final var measY = measurement.getY();
            final var measZ = measurement.getZ();

            final var deltaX = measX - xEstPropagated.getElementAtIndex(0);
            final var deltaY = measY - xEstPropagated.getElementAtIndex(1);
            final var deltaZ = measZ - xEstPropagated.getElementAtIndex(2);
            final var approxRange = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

            // Calculate frame rotation during signal transit time using (8.36)
            final var ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
            cei.setElementAt(0, 1, ceiValue);
            cei.setElementAt(1, 0, -ceiValue);

            // Predict pseudo-range using (9.165)
            satellitePosition.setElementAtIndex(0, measX);
            satellitePosition.setElementAtIndex(1, measY);
            satellitePosition.setElementAtIndex(2, measZ);

            cei.multiply(satellitePosition, deltaR);
            for (var i = 0; i < CoordinateTransformation.ROWS; i++) {
                deltaR.setElementAtIndex(i, deltaR.getElementAtIndex(i) - xEstPropagated.getElementAtIndex(i));
            }
            final var range = Utils.normF(deltaR);

            predMeas.setElementAt(j, 0, range + xEstPropagated.getElementAtIndex(6));

            // Predict line of sight
            for (var i = 0; i < CoordinateTransformation.ROWS; i++) {
                uAseT.setElementAt(j, i, deltaR.getElementAtIndex(i) / range);
            }

            // Predict pseudo-range rate using (9.165)
            satelliteVelocity.setElementAtIndex(0, measurement.getVx());
            satelliteVelocity.setElementAtIndex(1, measurement.getVy());
            satelliteVelocity.setElementAtIndex(2, measurement.getVz());

            omegaIe.multiply(satellitePosition, tmp1);
            satelliteVelocity.add(tmp1, tmp2);
            cei.multiply(tmp2, tmp3);

            omegaIe.multiply(propagatedPosition, tmp4);
            propagatedVelocity.add(tmp4, tmp6);

            tmp3.subtract(tmp6, tmp5);

            uAseT.getSubmatrix(j, 0, j, 2, tmp7);

            final var rangeRate = Utils.dotProduct(tmp7, tmp5);

            predMeas.setElementAt(j, 1, rangeRate + xEstPropagated.getElementAtIndex(7));

            j++;
        }

        // 5. Set-up measurement matrix using (9.163)
        final var h = new Matrix(2 * numberOfMeasurements, GNSSEstimation.NUM_PARAMETERS);
        for (int j1 = 0, j2 = numberOfMeasurements; j1 < numberOfMeasurements; j1++, j2++) {
            for (int i1 = 0, i2 = 3; i1 < CoordinateTransformation.ROWS; i1++, i2++) {
                final var value = -uAseT.getElementAt(j1, i1);

                h.setElementAt(j1, i1, value);
                h.setElementAt(j2, i2, value);
            }
            h.setElementAt(j1, 6, 1.0);
            h.setElementAt(j2, 7, 1.0);
        }

        // 6. Set-up measurement noise covariance matrix assuming all measurements are independent
        // and have equal variance for a given measurement type
        final var pseudoRangeSD = config.getPseudoRangeSD();
        final var pseudoRangeSD2 = pseudoRangeSD * pseudoRangeSD;
        final var rangeRateSD = config.getRangeRateSD();
        final var rangeRateSD2 = rangeRateSD * rangeRateSD;
        final var r = new Matrix(2 * numberOfMeasurements, 2 * numberOfMeasurements);
        for (int i1 = 0, i2 = numberOfMeasurements; i1 < numberOfMeasurements; i1++, i2++) {
            r.setElementAt(i1, i1, pseudoRangeSD2);
            r.setElementAt(i2, i2, rangeRateSD2);
        }

        // 7. Calculate Kalman gain using (3.21)
        final var hTransposed = h.transposeAndReturnNew();
        final var tmp8 = h.multiplyAndReturnNew(pMatrixPropagated.multiplyAndReturnNew(hTransposed));
        tmp8.add(r);
        final var tmp9 = Utils.inverse(tmp8);
        final var k = pMatrixPropagated.multiplyAndReturnNew(hTransposed);
        k.multiply(tmp9);

        // 8. Formulate measurement innovations using (3.88)
        final var deltaZ = new Matrix(2 * numberOfMeasurements, 1);
        var i1 = 0;
        var i2 = numberOfMeasurements;
        for (final var measurement : measurements) {
            deltaZ.setElementAtIndex(i1, measurement.getPseudoRange() - predMeas.getElementAt(i1, 0));
            deltaZ.setElementAtIndex(i2, measurement.getPseudoRate() - predMeas.getElementAt(i1, 1));
            i1++;
            i2++;
        }

        // 9. Update state estimates using (3.24)
        xEstPropagated.add(k.multiplyAndReturnNew(deltaZ));

        // xEstPropagated now contains updated state
        updatedEstimation.fromMatrix(xEstPropagated);

        // 10. Update state estimation error covariance matrix using (3.25)
        if (updatedCovariance.getRows() != GNSSEstimation.NUM_PARAMETERS
                || updatedCovariance.getColumns() != GNSSEstimation.NUM_PARAMETERS) {
            updatedCovariance.resize(GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);
        }
        Matrix.identity(updatedCovariance);
        k.multiply(h);
        updatedCovariance.subtract(k);
        updatedCovariance.multiply(pMatrixPropagated);
    }

    /**
     * Converts time instance into a value expressed in seconds.
     *
     * @param time time instance to be converted.
     * @return time value expressed in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
    }
}
