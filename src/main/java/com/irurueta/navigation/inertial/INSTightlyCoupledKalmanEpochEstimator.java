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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.Collection;

/**
 * Implements one cycle of the tightly coupled INS/GNSS
 * Kalman filter plus closed-loop correction of all inertial states.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/TC_KF_Epoch.m
 */
public class INSTightlyCoupledKalmanEpochEstimator {

    /**
     * Speed of light in the vacuum expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = Constants.SPEED_OF_LIGHT;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {

        // Skew symmetric matrix of Earth rate
        final Matrix omegaIe = Utils.skewMatrix(
                new double[]{0.0, 0.0, EARTH_ROTATION_RATE});

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (14.50) (first-order approx)
        final Matrix phiMatrix = Matrix.identity(
                INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final Matrix tmp1 = omegaIe.multiplyByScalarAndReturnNew(
                propagationInterval);
        final Matrix tmp2 = phiMatrix.getSubmatrix(0, 0,
                2, 2);
        tmp2.subtract(tmp1);

        phiMatrix.setSubmatrix(0, 0,
                2, 2, tmp2);

        final Matrix estCbeOld = previousState
                .getBodyToEcefCoordinateTransformationMatrix();
        tmp1.copyFrom(estCbeOld);
        tmp1.multiplyByScalar(propagationInterval);

        phiMatrix.setSubmatrix(0, 12,
                2, 14, tmp1);
        phiMatrix.setSubmatrix(3, 9,
                5, 11, tmp1);

        final Matrix measFibb = new Matrix(BodyKinematics.COMPONENTS, 1);
        measFibb.setElementAtIndex(0, fx);
        measFibb.setElementAtIndex(1, fy);
        measFibb.setElementAtIndex(2, fz);

        estCbeOld.multiply(measFibb, tmp1);

        Utils.skewMatrix(tmp1, tmp2);
        tmp2.multiplyByScalar(-propagationInterval);

        phiMatrix.setSubmatrix(3, 0,
                5, 2, tmp2);

        phiMatrix.getSubmatrix(3, 3,
                5, 5, tmp1);
        tmp2.copyFrom(omegaIe);
        tmp2.multiplyByScalar(2.0 * propagationInterval);
        tmp1.subtract(tmp2);
        phiMatrix.setSubmatrix(3, 3,
                5, 5, tmp1);

        final double sinPrevLat = Math.sin(previousLatitude);
        final double cosPrevLat = Math.cos(previousLatitude);
        final double sinPrevLat2 = sinPrevLat * sinPrevLat;
        final double cosPrevLat2 = cosPrevLat * cosPrevLat;

        // From (2.137)
        final double geocentricRadius = EARTH_EQUATORIAL_RADIUS_WGS84
                / Math.sqrt(1.0 - Math.pow(EARTH_ECCENTRICITY * sinPrevLat, 2.0))
                * Math.sqrt(cosPrevLat2
                + Math.pow(1.0 - EARTH_ECCENTRICITY * EARTH_ECCENTRICITY, 2.0) * sinPrevLat2);

        final double prevX = previousState.getX();
        final double prevY = previousState.getY();
        final double prevZ = previousState.getZ();
        final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                prevX, prevY, prevZ);

        final double previousPositionNorm = Math.sqrt(prevX * prevX +
                prevY * prevY + prevZ * prevZ);

        final Matrix estRebeOld = new Matrix(ECEFPosition.COMPONENTS, 1);
        estRebeOld.setElementAtIndex(0, prevX);
        estRebeOld.setElementAtIndex(1, prevY);
        estRebeOld.setElementAtIndex(2, prevZ);

        final Matrix g = gravity.asMatrix();
        g.multiplyByScalar(-2.0 * propagationInterval / geocentricRadius);

        final Matrix estRebeOldTrans = estRebeOld.transposeAndReturnNew();
        estRebeOldTrans.multiplyByScalar(1.0 / previousPositionNorm);

        g.multiply(estRebeOldTrans, tmp1);

        phiMatrix.setSubmatrix(3, 6,
                5, 8, tmp1);

        for (int i = 0; i < ECEFPosition.COMPONENTS; i++) {
            phiMatrix.setElementAt(6 + i, 3 + i, propagationInterval);
        }

        phiMatrix.setElementAt(15, 16, propagationInterval);


        // 2. Determine approximate system noise covariance matrix using (14.82)
        final Matrix qPrimeMatrix = new Matrix(
                INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final double gyroNoisePSD = config.getGyroNoisePSD();
        final double gyroNoiseValue = gyroNoisePSD * propagationInterval;
        for (int i = 0; i < 3; i++) {
            qPrimeMatrix.setElementAt(i, i, gyroNoiseValue);
        }

        final double accelNoisePSD = config.getAccelerometerNoisePSD();
        final double accelNoiseValue = accelNoisePSD * propagationInterval;
        for (int i = 3; i < 6; i++) {
            qPrimeMatrix.setElementAt(i, i, accelNoiseValue);
        }

        final double accelBiasPSD = config.getAccelerometerBiasPSD();
        final double accelBiasValue = accelBiasPSD * propagationInterval;
        for (int i = 9; i < 12; i++) {
            qPrimeMatrix.setElementAt(i, i, accelBiasValue);
        }

        final double gyroBiasPSD = config.getGyroBiasPSD();
        final double gyroBiasValue = gyroBiasPSD * propagationInterval;
        for (int i = 12; i < 15; i++) {
            qPrimeMatrix.setElementAt(i, i, gyroBiasValue);
        }

        final double clockPhasePSD = config.getClockPhasePSD();
        final double clockPhaseValue = clockPhasePSD * propagationInterval;
        qPrimeMatrix.setElementAt(15, 15, clockPhaseValue);

        final double clockFreqPSD = config.getClockFrequencyPSD();
        final double clockFreqValue = clockFreqPSD * propagationInterval;
        qPrimeMatrix.setElementAt(16, 16, clockFreqValue);


        // 3. Propagate state estimates using (3.14) noting that only the clock
        // states are non-zero due to closed-loop correction
        final double prevClockOffset = previousState.getReceiverClockOffset();
        final double prevClockDrift = previousState.getReceiverClockDrift();

        final Matrix xEstPropagated = new Matrix(INSTightlyCoupledKalmanState.NUM_PARAMS, 1);
        xEstPropagated.setElementAtIndex(15, prevClockOffset
                + prevClockDrift * propagationInterval);
        xEstPropagated.setElementAtIndex(16, prevClockDrift);

        // 4. Propagate state estimation error covariance matrix using (3.46)
        final Matrix pMatrixOld = previousState.getCovariance();

        qPrimeMatrix.multiplyByScalar(0.5);

        final Matrix tmp3 = pMatrixOld.addAndReturnNew(qPrimeMatrix);
        final Matrix pMatrixPropagated = phiMatrix.multiplyAndReturnNew(tmp3);

        phiMatrix.transpose();
        pMatrixPropagated.multiply(phiMatrix);

        pMatrixPropagated.add(qPrimeMatrix);


        // MEASUREMENT UPDATE PHASE

        final int numberOfMeasurements = measurements.size();
        final Matrix uAseT = new Matrix(numberOfMeasurements, 3);
        final Matrix predMeas = new Matrix(numberOfMeasurements, 2);

        final Matrix cei = Matrix.identity(CoordinateTransformation.ROWS,
                CoordinateTransformation.COLS);
        final Matrix satellitePosition = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix satelliteVelocity = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix deltaR = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp1b = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp2b = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp3b = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp4b = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp5b = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp6b = new Matrix(CoordinateTransformation.ROWS, 1);
        final Matrix tmp7b = new Matrix(1, CoordinateTransformation.ROWS);

        final double prevVx = previousState.getVx();
        final double prevVy = previousState.getVy();
        final double prevVz = previousState.getVz();

        final Matrix estVebeOld = new Matrix(ECEFVelocity.COMPONENTS, 1);
        estVebeOld.setElementAtIndex(0, prevVx);
        estVebeOld.setElementAtIndex(1, prevVy);
        estVebeOld.setElementAtIndex(2, prevVz);

        int j = 0;
        for (final GNSSMeasurement measurement : measurements) {
            // Predict approx range
            final double measX = measurement.getX();
            final double measY = measurement.getY();
            final double measZ = measurement.getZ();

            final double deltaX = measX - prevX;
            final double deltaY = measY - prevY;
            final double deltaZ = measZ - prevZ;
            final double approxRange = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

            // Calculate frame rotation during signal transit time using (8.36)
            final double ceiValue = EARTH_ROTATION_RATE * approxRange / SPEED_OF_LIGHT;
            cei.setElementAt(0, 1, ceiValue);
            cei.setElementAt(1, 0, -ceiValue);

            // Predict pseudo-range using (9.165)
            satellitePosition.setElementAtIndex(0, measX);
            satellitePosition.setElementAtIndex(1, measY);
            satellitePosition.setElementAtIndex(2, measZ);

            cei.multiply(satellitePosition, deltaR);
            for (int i = 0; i < CoordinateTransformation.ROWS; i++) {
                deltaR.setElementAtIndex(i, deltaR.getElementAtIndex(i)
                        - estRebeOld.getElementAtIndex(i));
            }
            final double range = Utils.normF(deltaR);

            predMeas.setElementAt(j, 0, range
                    + xEstPropagated.getElementAtIndex(15));

            // Predict line of sight
            for (int i = 0; i < CoordinateTransformation.ROWS; i++) {
                uAseT.setElementAt(j, i, deltaR.getElementAtIndex(i) / range);
            }

            // Predict pseudo-range rae using (9.165)
            satelliteVelocity.setElementAtIndex(0, measurement.getVx());
            satelliteVelocity.setElementAtIndex(1, measurement.getVy());
            satelliteVelocity.setElementAtIndex(2, measurement.getVz());

            omegaIe.multiply(satellitePosition, tmp1b);
            satelliteVelocity.add(tmp1b, tmp2b);
            cei.multiply(tmp2b, tmp3b);

            omegaIe.multiply(estRebeOld, tmp4b);
            estVebeOld.add(tmp4b, tmp6b);

            tmp3b.subtract(tmp6b, tmp5b);

            uAseT.getSubmatrix(j, 0, j, 2, tmp7b);

            final double rangeRate = Utils.dotProduct(tmp7b, tmp5b);

            predMeas.setElementAt(j, 1, rangeRate
                    + xEstPropagated.getElementAtIndex(16));

            j++;
        }

        // 5. Set-up measurement matrix using (14.126)
        final Matrix h = new Matrix(2 * numberOfMeasurements, INSTightlyCoupledKalmanState.NUM_PARAMS);
        for (int j1 = 0, j2 = numberOfMeasurements; j1 < numberOfMeasurements; j1++, j2++) {
            for (int i1 = 0, i2 = 6, i3 = 3; i1 < CoordinateTransformation.ROWS; i1++, i2++, i3++) {
                final double value = uAseT.getElementAt(j1, i1);

                h.setElementAt(j1, i2, value);
                h.setElementAt(j2, i3, value);
            }
            h.setElementAt(j1, 15, 1.0);
            h.setElementAt(j2, 16, 1.0);
        }

        // 6. Set-up measurement noise covariance matrix assuming all measurements are independent
        // and have equal variance for a given measurement type.
        final double pseudoRangeSD = config.getPseudoRangeSD();
        final double pseudoRangeSD2 = pseudoRangeSD * pseudoRangeSD;
        final double rangeRateSD = config.getRangeRateSD();
        final double rangeRateSD2 = rangeRateSD * rangeRateSD;
        final Matrix r = new Matrix(2 * numberOfMeasurements, 2 * numberOfMeasurements);
        for (int i1 = 0, i2 = numberOfMeasurements; i1 < numberOfMeasurements; i1++, i2++) {
            r.setElementAt(i1, i1, pseudoRangeSD2);
            r.setElementAt(i2, i2, rangeRateSD2);
        }

        // 7. Calculate Kalman gain using (3.21)
        final Matrix hTransposed = h.transposeAndReturnNew();
        final Matrix tmp8b = h.multiplyAndReturnNew(
                pMatrixPropagated.multiplyAndReturnNew(hTransposed));
        tmp8b.add(r);
        final Matrix tmp9b = Utils.inverse(tmp8b);
        final Matrix k = pMatrixPropagated.multiplyAndReturnNew(hTransposed);
        k.multiply(tmp9b);

        // 8. Formulate measurement innovations using (14.119)
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
        xEstPropagated.add(k.multiplyAndReturnNew(deltaZ));

        // xEstPropagated now contains updated state

        // 10. Update state estimation error covariance matrix using (3.25)
        Matrix updatedCovariance = result.getCovariance();
        if (updatedCovariance == null ||
                updatedCovariance.getRows() != INSTightlyCoupledKalmanState.NUM_PARAMS ||
                updatedCovariance.getColumns() != INSTightlyCoupledKalmanState.NUM_PARAMS) {
            updatedCovariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                    INSTightlyCoupledKalmanState.NUM_PARAMS);
        } else {
            Matrix.identity(updatedCovariance);
        }
        k.multiply(h);
        updatedCovariance.subtract(k);
        updatedCovariance.multiply(pMatrixPropagated);


        // CLOSED-LOOP CORRECTION

        // Correct attitude, velocity, and position using (14.7-9)

        final Matrix estCbeNew = Matrix.identity(CoordinateTransformation.ROWS,
                CoordinateTransformation.COLS);

        xEstPropagated.getSubmatrix(0, 0,
                2, 0, tmp1b);
        estCbeNew.subtract(Utils.skewMatrix(tmp1b));
        estCbeNew.multiply(estCbeOld);


        result.setBodyToEcefCoordinateTransformationMatrix(estCbeNew);
        result.setVelocityCoordinates(
                prevVx - xEstPropagated.getElementAtIndex(3),
                prevVy - xEstPropagated.getElementAtIndex(4),
                prevVz - xEstPropagated.getElementAtIndex(5));
        result.setPositionCoordinates(
                prevX - xEstPropagated.getElementAtIndex(6),
                prevY - xEstPropagated.getElementAtIndex(7),
                prevZ - xEstPropagated.getElementAtIndex(8));
        result.setCovariance(updatedCovariance);

        // Update IMU bias and GNSS receiver clock estimates
        final double prevAccelBiasX = previousState.getAccelerationBiasX();
        final double prevAccelBiasY = previousState.getAccelerationBiasY();
        final double prevAccelBiasZ = previousState.getAccelerationBiasZ();
        final double prevGyroBiasX = previousState.getGyroBiasX();
        final double prevGyroBiasY = previousState.getGyroBiasY();
        final double prevGyroBiasZ = previousState.getGyroBiasZ();

        result.setAccelerationBiasCoordinates(
                prevAccelBiasX + xEstPropagated.getElementAtIndex(9),
                prevAccelBiasY + xEstPropagated.getElementAtIndex(10),
                prevAccelBiasZ + xEstPropagated.getElementAtIndex(11));
        result.setGyroBiasCoordinates(
                prevGyroBiasX + xEstPropagated.getElementAtIndex(12),
                prevGyroBiasY + xEstPropagated.getElementAtIndex(13),
                prevGyroBiasZ + xEstPropagated.getElementAtIndex(14));

        result.setReceiverClockOffset(xEstPropagated.getElementAtIndex(15));
        result.setReceiverClockDrift(xEstPropagated.getElementAtIndex(16));
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        final INSTightlyCoupledKalmanState result = new INSTightlyCoupledKalmanState();
        estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, convertTime(propagationInterval),
                previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, convertTime(propagationInterval),
                previousState, fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {

        final NEDPosition prevNedPosition = new NEDPosition();
        final NEDVelocity prevNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                previousState.getX(), previousState.getY(), previousState.getZ(),
                previousState.getVx(), previousState.getVy(), previousState.getVz(),
                prevNedPosition, prevNedVelocity);

        final double previousLatitude = prevNedPosition.getLatitude();

        estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        final INSTightlyCoupledKalmanState result = new INSTightlyCoupledKalmanState();
        estimate(measurements, propagationInterval, previousState, fx, fy, fz,
                config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, convertTime(propagationInterval),
                previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            second (m/s^2).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, convertTime(propagationInterval),
                previousState, fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {

        final double fx = bodyKinematics.getFx();
        final double fy = bodyKinematics.getFy();
        final double fz = bodyKinematics.getFz();

        estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        final INSTightlyCoupledKalmanState result = new INSTightlyCoupledKalmanState();
        estimate(measurements, propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, convertTime(propagationInterval),
                previousState, bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, convertTime(propagationInterval),
                previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {

        final double fx = bodyKinematics.getFx();
        final double fy = bodyKinematics.getFy();
        final double fz = bodyKinematics.getFz();

        estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        final INSTightlyCoupledKalmanState result = new INSTightlyCoupledKalmanState();
        estimate(measurements, propagationInterval, previousState, bodyKinematics,
                config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, convertTime(propagationInterval), previousState,
                bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, convertTime(propagationInterval),
                previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final double propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config);
    }

    /**
     * Estimtes the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config,
            final INSTightlyCoupledKalmanState result) throws AlgebraException {
        estimate(measurements, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state and covariance matrix for a single
     * epoch.
     *
     * @param measurements        satellite measurements data.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Tightly Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSTightlyCoupledKalmanState estimate(
            final Collection<GNSSMeasurement> measurements,
            final Time propagationInterval,
            final INSTightlyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(measurements, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config);
    }

    /**
     * Converts time instance into a value expressed in seconds.
     *
     * @param time time instance to be converted.
     * @return time value expressed in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Converts angle instance into a value expressed in radians.
     *
     * @param angle angle instance to be converted.
     * @return angle value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }
}
