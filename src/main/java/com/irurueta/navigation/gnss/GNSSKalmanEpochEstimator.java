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

import java.util.Collection;


/**
 * Implements one cycle of the GNSS extended Kalman filter.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
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
     * Estimates the update of Kalman filter state and covariance matrix for a single epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state estimates.
     * @param previousCovariance  previous Kalman filter error covariance matrix.
     * @param config              system configuration (usually obtained through calibration).
     * @param updatedState        instance where updated Kalman filter state will be stored after executing this method.
     * @param updatedCovariance   instance where updated Kalman filter error covariance matrix will be stored.
     * @throws IllegalArgumentException if provided previous covariance matrix is not 8x8.
     */
    public static void estimate(final Collection<GNSSMeasurement> measurements, final double propagationInterval,
                                final GNSSEstimation previousState, final Matrix previousCovariance,
                                final GNSSKalmanConfig config, GNSSEstimation updatedState,
                                final Matrix updatedCovariance) throws AlgebraException {

        if (previousCovariance.getRows() != GNSSEstimation.NUM_PARAMETERS ||
                previousCovariance.getColumns() != GNSSEstimation.NUM_PARAMETERS) {
            throw new IllegalArgumentException();
        }

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (9.147) and (9.150)
        final Matrix phiMatrix = Matrix.identity(MATRIX_SIZE, MATRIX_SIZE);
        phiMatrix.setElementAt(0, 3, propagationInterval);
        phiMatrix.setElementAt(1, 4, propagationInterval);
        phiMatrix.setElementAt(2, 5, propagationInterval);
        phiMatrix.setElementAt(6, 7, propagationInterval);

        // 2. Determine system noise covariance matrix using (9.152)
        final double propagationInterval2 = propagationInterval * propagationInterval;
        final double propagationInterval3 = propagationInterval2 * propagationInterval;
        final double accelerationPSD = config.getAccelerationPSD();
        final double clockFrequencyPSD = config.getClockFrequencyPSD();
        final double clockPhasePSD = config.getClockPhasePSD();

        final double value1 = accelerationPSD * propagationInterval3 / 3.0;
        final double value2 = accelerationPSD * propagationInterval2 / 2.0;
        final double value3 = accelerationPSD * propagationInterval;
        final double value4 = clockFrequencyPSD * propagationInterval3 / 3.0 + clockPhasePSD * propagationInterval;
        final double value5 = clockFrequencyPSD * propagationInterval2 / 2.0;
        final double value6 = clockFrequencyPSD * propagationInterval;

        final Matrix qMatrix = new Matrix(MATRIX_SIZE, MATRIX_SIZE);
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
        final Matrix xEstOld = previousState.asMatrix();
        final Matrix xEstPropagated = phiMatrix.multiplyAndReturnNew(xEstOld);
        final Matrix propagatedVelocity = xEstPropagated.getSubmatrix(
                3, 0, 5, 0);
        final Matrix propagatedPosition = xEstPropagated.getSubmatrix(
                0, 0, 2, 0);

        // 4. Propagate state estimation error covariance matrix using (3.15)
        final Matrix pMatrixPropagated = phiMatrix.multiplyAndReturnNew(previousCovariance);
        phiMatrix.transpose();
        pMatrixPropagated.multiply(phiMatrix);
        pMatrixPropagated.add(qMatrix);

        // MEASUREMENT UPDATE PHASE

        // Skew symmetric matrix of Earth rate
        final Matrix omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, EARTH_ROTATION_RATE});

        final int numberOfMeasurements = measurements.size();
        final Matrix uAseT = new Matrix(numberOfMeasurements, 3);
        final Matrix predMeas = new Matrix(numberOfMeasurements, 2);

        final Matrix cei = Matrix.identity(CoordinateTransformation.ROWS,
                CoordinateTransformation.COLS);
        final Matrix satellitePosition = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix satelliteVelocity = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix deltaR = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp1 = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp2 = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp3 = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp4 = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp5 = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp6 = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp7 = new Matrix(1, CoordinateTransformation.ROWS);

        // Loop measurements
        int j = 0;
        for (final GNSSMeasurement measurement : measurements) {
            // Predict approx range
            final double deltaX = measurement.getX() - xEstPropagated.getElementAtIndex(0);
            final double deltaY = measurement.getY() - xEstPropagated.getElementAtIndex(1);
            final double deltaZ = measurement.getZ() - xEstPropagated.getElementAtIndex(2);
            final double approxRange = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

            // Calculate frame rotation during signal transit time using (8.36)
            final double ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
            cei.setElementAt(0, 1, ceiValue);
            cei.setElementAt(1, 0, -ceiValue);

            // Predict pseudo-range using (9.165)
            satellitePosition.setElementAtIndex(0, measurement.getX());
            satellitePosition.setElementAtIndex(1, measurement.getY());
            satellitePosition.setElementAtIndex(2, measurement.getZ());

            cei.multiply(satellitePosition, deltaR);
            for (int i = 0; i < deltaR.getRows(); i++) {
                deltaR.setElementAtIndex(i, deltaR.getElementAtIndex(i)
                        - xEstPropagated.getElementAtIndex(i));
            }
            final double range = Utils.normF(deltaR);

            predMeas.setElementAt(j, 0, range
                    + xEstPropagated.getElementAtIndex(6));

            // Predict line of sight
            for (int i = 0; i < deltaR.getRows(); i++) {
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

            double rangeRate = Utils.dotProduct(tmp7, tmp5);

            predMeas.setElementAt(j, 1,
                    rangeRate + xEstPropagated.getElementAtIndex(7));

            j++;
        }

        // 5. Set-up measurement matrix using (9.163)
        final Matrix hMatrix = new Matrix(2 * numberOfMeasurements, GNSSEstimation.NUM_PARAMETERS);
        for (int j1 = 0, j2 = numberOfMeasurements; j1 < numberOfMeasurements; j1++, j2++) {
            for (int i1 = 0, i2 = 3; i1 < CoordinateTransformation.ROWS; i1++, i2++) {
                final double value = -uAseT.getElementAt(j1, i1);

                hMatrix.setElementAt(j1, i1, value);
                hMatrix.setElementAt(j2, i2, value);
            }
            hMatrix.setElementAt(j1, 6, 1.0);
            hMatrix.setElementAt(j2, 7, 1.0);
        }

        // 6. Set-up measurement noise covariance matrix assuming all measurements are independent
        // and have equal variance for a given measurement type
        final double pseudoRangeSD = config.getPseudoRangeSD();
        final double pseudoRangeSD2 = pseudoRangeSD * pseudoRangeSD;
        final double rangeRateSD = config.getRangeRateSD();
        final double rangeRateSD2 = rangeRateSD * rangeRateSD;
        final Matrix rMatrix = new Matrix(2 * numberOfMeasurements, 2 * numberOfMeasurements);
        for (int i1 = 0, i2 = numberOfMeasurements; i1 < numberOfMeasurements; i1++, i2++) {
            rMatrix.setElementAt(i1, i1, pseudoRangeSD2);
            rMatrix.setElementAt(i2, i2, rangeRateSD2);
        }

        // 7. Calculate Kalman gain using (3.21)
        final Matrix hMatrixTransposed = hMatrix.transposeAndReturnNew();
        final Matrix tmp8 = hMatrix.multiplyAndReturnNew(
                pMatrixPropagated.multiplyAndReturnNew(hMatrixTransposed));
        tmp8.add(rMatrix);
        final Matrix tmp9 = Utils.inverse(tmp8);
        final Matrix kMatrix = pMatrixPropagated.multiplyAndReturnNew(hMatrixTransposed);
        kMatrix.multiply(tmp9);

        // 8. Formulate measurement innovations using (3.88)
        final Matrix deltaZ = new Matrix(2 * numberOfMeasurements, 1);
        int i1 = 0;
        int i2 = numberOfMeasurements;
        for (final GNSSMeasurement measurement : measurements) {
            deltaZ.setElementAtIndex(i1, measurement.getPseudoRange()
                    - predMeas.getElementAt(i1, 0));
            deltaZ.setElementAtIndex(i2, measurement.getPseudoRate()
                    - predMeas.getElementAt(i1, 1));
            i1++;
            i2++;
        }

        // 9. Update state estimates using (3.24)
        xEstPropagated.add(kMatrix.multiplyAndReturnNew(deltaZ));

        // xEstPropagated now contains updated state
        updatedState.fromMatrix(xEstPropagated);

        // 10. Update state estimation error covariance matrix using (3.25)
        if (updatedCovariance.getRows() != GNSSEstimation.NUM_PARAMETERS ||
                updatedCovariance.getColumns() != GNSSEstimation.NUM_PARAMETERS) {
            updatedCovariance.resize(GNSSEstimation.NUM_PARAMETERS,
                    GNSSEstimation.NUM_PARAMETERS);
        }
        Matrix.identity(updatedCovariance);
        kMatrix.multiply(hMatrix);
        updatedCovariance.subtract(kMatrix);
        updatedCovariance.multiply(pMatrixPropagated);
    }
}
