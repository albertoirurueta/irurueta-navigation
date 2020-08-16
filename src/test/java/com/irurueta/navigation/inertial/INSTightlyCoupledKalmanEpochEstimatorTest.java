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
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.GNSSKalmanEpochEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class INSTightlyCoupledKalmanEpochEstimatorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -50.0;
    private static final double MAX_HEIGHT_METERS = 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final int MIN_MEASUREMENTS = 4;
    private static final int MAX_MEASUREMENTS = 10;

    private static final double MIN_SAT_HEIGHT_METERS = 150000;
    private static final double MAX_SAT_HEIGHT_METERS = 500000;

    private static final double MIN_SAT_SPEED_VALUE = -20.0;
    private static final double MAX_SAT_SPEED_VALUE = 20.0;

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    public void testEstimate() throws AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double userLatitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(
                    MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

            final double userVn = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
            final double userVe = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
            final double userVd = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

            final NEDPosition userNedPosition = new NEDPosition(
                    userLatitude, userLongitude, userHeight);
            final NEDVelocity userNedVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition userEcefPosition = new ECEFPosition();
            final ECEFVelocity userEcefVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    userNedPosition, userNedVelocity,
                    userEcefPosition, userEcefVelocity);

            final NEDFrame userNedFrame = new NEDFrame(
                    userLatitude, userLongitude, userHeight, userVn, userVe, userVd);
            final ECEFFrame userEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(userNedFrame);

            final Point3D userPosition = userEcefFrame.getPosition();

            final int numMeasurements = randomizer.nextInt(MIN_MEASUREMENTS, MAX_MEASUREMENTS);

            final List<GNSSMeasurement> measurements = new ArrayList<>();
            for (int i = 0; i < numMeasurements; i++) {
                final double satLatitude = Math.toRadians(randomizer.nextDouble(
                        MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
                final double satLongitude = Math.toRadians(randomizer.nextDouble(
                        MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
                final double satHeight = randomizer.nextDouble(
                        MIN_SAT_HEIGHT_METERS, MAX_SAT_HEIGHT_METERS);

                final double satVn = randomizer.nextDouble(
                        MIN_SAT_SPEED_VALUE, MAX_SAT_SPEED_VALUE);
                final double satVe = randomizer.nextDouble(
                        MIN_SAT_SPEED_VALUE, MAX_SAT_SPEED_VALUE);
                final double satVd = randomizer.nextDouble(
                        MIN_SAT_SPEED_VALUE, MAX_SAT_SPEED_VALUE);

                final NEDFrame satNedFrame = new NEDFrame(satLatitude, satLongitude, satHeight,
                        satVn, satVe, satVd);
                final ECEFFrame satEcefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(satNedFrame);

                final Point3D satPosition = satEcefFrame.getPosition();

                final double pseudoRange = userPosition.distanceTo(satPosition);

                final double[] posDiff = new double[]{
                        satEcefFrame.getX() - userEcefFrame.getX(),
                        satEcefFrame.getY() - userEcefFrame.getY(),
                        satEcefFrame.getZ() - userEcefFrame.getZ()};
                final double posNorm = Utils.normF(posDiff);

                final double[] velDiff = new double[]{
                        satEcefFrame.getVx() - userEcefFrame.getVx(),
                        satEcefFrame.getVy() - userEcefFrame.getVy(),
                        satEcefFrame.getVz() - userEcefFrame.getVz()};
                final double velNorm = Utils.normF(velDiff);

                final double dot = Utils.dotProduct(posDiff, velDiff);
                final double cosAngle = dot / (posNorm * velNorm);

                final double pseudoRate = velNorm * cosAngle;

                final double x = satEcefFrame.getX();
                final double y = satEcefFrame.getY();
                final double z = satEcefFrame.getZ();

                final double vx = satEcefFrame.getVx();
                final double vy = satEcefFrame.getVy();
                final double vz = satEcefFrame.getVz();

                measurements.add(new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z,
                        vx, vy, vz));
            }

            final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

            final Matrix covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                    INSTightlyCoupledKalmanState.NUM_PARAMS);

            final BodyKinematics bodyKinematics = new BodyKinematics();
            final double fx = bodyKinematics.getFx();
            final double fy = bodyKinematics.getFy();
            final double fz = bodyKinematics.getFz();

            final ECEFPosition previousPosition = new ECEFPosition(
                    userEcefPosition.getX() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVx(),
                    userEcefPosition.getY() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVy(),
                    userEcefPosition.getZ() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVz());

            final NEDPosition previousNedPosition = new NEDPosition();
            final NEDVelocity previousNedVelocity = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(previousPosition,
                    userEcefVelocity, previousNedPosition, previousNedVelocity);
            final double previousLatitude = previousNedPosition.getLatitude();

            final INSTightlyCoupledKalmanState previousState =
                    new INSTightlyCoupledKalmanState(userEcefFrame.getCoordinateTransformation(),
                            userEcefVelocity, previousPosition,
                            accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                            gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset,
                            receiverClockDrift, covariance);


            final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double accelerometerNoisePSD =
                    randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double accelerometerBiasPSD =
                    randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig(
                    gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                    gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD,
                    rangeRateSD);

            final INSTightlyCoupledKalmanState result1 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitude,
                    config, result1);

            final INSTightlyCoupledKalmanState result2 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                            fx, fy, fz, previousLatitude, config);

            final Time propagationInterval = new Time(TIME_INTERVAL_SECONDS,
                    TimeUnit.SECOND);

            final INSTightlyCoupledKalmanState result3 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    propagationInterval, previousState, fx, fy, fz, previousLatitude,
                    config, result3);

            final INSTightlyCoupledKalmanState result4 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, propagationInterval, previousState,
                            fx, fy, fz, previousLatitude, config);

            final INSTightlyCoupledKalmanState result5 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, config, result5);

            final INSTightlyCoupledKalmanState result6 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                            fx, fy, fz, config);

            final INSTightlyCoupledKalmanState result7 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    propagationInterval, previousState, fx, fy, fz, config,
                    result7);

            final INSTightlyCoupledKalmanState result8 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, propagationInterval, previousState,
                            fx, fy, fz, config);

            final INSTightlyCoupledKalmanState result9 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics,
                    previousLatitude, config, result9);

            final INSTightlyCoupledKalmanState result10 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                            bodyKinematics, previousLatitude, config);

            final INSTightlyCoupledKalmanState result11 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, bodyKinematics, previousLatitude, config, result11);

            final INSTightlyCoupledKalmanState result12 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, propagationInterval, previousState,
                            bodyKinematics, previousLatitude, config);

            final INSTightlyCoupledKalmanState result13 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics, config,
                    result13);

            final INSTightlyCoupledKalmanState result14 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                            bodyKinematics, config);

            final INSTightlyCoupledKalmanState result15 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, bodyKinematics, config, result15);

            final INSTightlyCoupledKalmanState result16 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, propagationInterval, previousState, bodyKinematics,
                            config);

            final Angle previousLatitudeAngle = new Angle(previousLatitude,
                    AngleUnit.RADIANS);
            final INSTightlyCoupledKalmanState result17 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    TIME_INTERVAL_SECONDS, previousState, fx, fy, fz, previousLatitudeAngle,
                    config, result17);

            final INSTightlyCoupledKalmanState result18 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                            fx, fy, fz, previousLatitudeAngle, config);

            final INSTightlyCoupledKalmanState result19 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, fx, fy, fz, previousLatitudeAngle, config, result19);

            final INSTightlyCoupledKalmanState result20 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, propagationInterval, previousState,
                            fx, fy, fz, previousLatitudeAngle, config);

            final INSTightlyCoupledKalmanState result21 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements,
                    TIME_INTERVAL_SECONDS, previousState, bodyKinematics,
                    previousLatitudeAngle, config, result21);

            final INSTightlyCoupledKalmanState result22 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, TIME_INTERVAL_SECONDS, previousState, bodyKinematics,
                            previousLatitudeAngle, config);

            final INSTightlyCoupledKalmanState result23 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, bodyKinematics, previousLatitudeAngle, config, result23);

            final INSTightlyCoupledKalmanState result24 = INSTightlyCoupledKalmanEpochEstimator
                    .estimate(measurements, propagationInterval, previousState, bodyKinematics,
                            previousLatitudeAngle, config);

            assertEquals(result1, result2);
            assertEquals(result1, result3);
            assertEquals(result1, result4);
            assertEquals(result1, result5);
            assertEquals(result1, result6);
            assertEquals(result1, result7);
            assertEquals(result1, result8);
            assertEquals(result1, result9);
            assertEquals(result1, result10);
            assertEquals(result1, result11);
            assertEquals(result1, result12);
            assertEquals(result1, result13);
            assertEquals(result1, result14);
            assertEquals(result1, result15);
            assertEquals(result1, result16);
            assertEquals(result1, result17);
            assertEquals(result1, result18);
            assertEquals(result1, result19);
            assertEquals(result1, result20);
            assertEquals(result1, result21);
            assertEquals(result1, result22);
            assertEquals(result1, result23);
            assertEquals(result1, result24);

            final INSTightlyCoupledKalmanState expected = estimate(
                    measurements, previousState,
                    fx, fy, fz, previousLatitude, config);

            if (!expected.equals(result1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(expected.equals(result1, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static INSTightlyCoupledKalmanState estimate(
            final List<GNSSMeasurement> measurements,
            final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {

        // Skew symmetric matrix of Earth rate
        final Matrix omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, Constants.EARTH_ROTATION_RATE});

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (14.50) (first-order approx)
        final Matrix phiMatrix = Matrix.identity(17, 17);
        phiMatrix.setSubmatrix(0, 0,
                2, 2,
                phiMatrix.getSubmatrix(0, 0,
                        2, 2).subtractAndReturnNew(
                        omegaIe.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS)));

        final Matrix estCbeOld = previousState.getBodyToEcefCoordinateTransformationMatrix();
        phiMatrix.setSubmatrix(0, 12,
                2, 14,
                estCbeOld.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        final Matrix measFibb = Matrix.newFromArray(new double[]{fx, fy, fz});
        phiMatrix.setSubmatrix(3, 0,
                5, 2,
                Utils.skewMatrix(estCbeOld.multiplyAndReturnNew(measFibb))
                        .multiplyByScalarAndReturnNew(-TIME_INTERVAL_SECONDS));

        phiMatrix.setSubmatrix(3, 3,
                5, 5,
                phiMatrix.getSubmatrix(3, 3,
                        5, 5).subtractAndReturnNew(
                        omegaIe.multiplyByScalarAndReturnNew(
                                2.0 * TIME_INTERVAL_SECONDS)));

        final double sinPrevLat = Math.sin(previousLatitude);
        final double cosPrevLat = Math.cos(previousLatitude);
        final double sinPrevLat2 = sinPrevLat * sinPrevLat;
        final double cosPrevLat2 = cosPrevLat * cosPrevLat;

        final double geocentricRadius = Constants.EARTH_EQUATORIAL_RADIUS_WGS84
                / Math.sqrt(1.0 - Math.pow(Constants.EARTH_ECCENTRICITY * sinPrevLat, 2.0))
                * Math.sqrt(cosPrevLat2
                + Math.pow(1.0 - Constants.EARTH_ECCENTRICITY * Constants.EARTH_ECCENTRICITY, 2.0)
                * sinPrevLat2);

        final double prevX = previousState.getX();
        final double prevY = previousState.getY();
        final double prevZ = previousState.getZ();
        final ECEFGravity gravity = ECEFGravityEstimator
                .estimateGravityAndReturnNew(prevX, prevY, prevZ);
        final Matrix g = gravity.asMatrix();

        final Matrix estRebeOld = Matrix.newFromArray(new double[]{prevX, prevY, prevZ});

        final Matrix gScaled = g.multiplyByScalarAndReturnNew(
                -2.0 * TIME_INTERVAL_SECONDS / geocentricRadius);
        final Matrix estRebeOldTrans = estRebeOld.transposeAndReturnNew();
        final double previousPositionNorm = Math.sqrt(prevX * prevX +
                prevY * prevY + prevZ * prevZ);
        final Matrix estRebeOldTransScaled = estRebeOldTrans.multiplyByScalarAndReturnNew(
                1.0 / previousPositionNorm);
        phiMatrix.setSubmatrix(3, 6,
                5, 8,
                gScaled.multiplyAndReturnNew(estRebeOldTransScaled));

        phiMatrix.setSubmatrix(3, 9,
                5, 11,
                estCbeOld.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        phiMatrix.setSubmatrix(6, 3,
                8, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        phiMatrix.setElementAt(15, 16, TIME_INTERVAL_SECONDS);

        // 2. Determine approximate system noise covariance matrix using (14.82)
        final Matrix qPrimeMatrix = new Matrix(17, 17);
        qPrimeMatrix.setSubmatrix(0, 0,
                2, 2,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getGyroNoisePSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(3, 3,
                5, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getAccelerometerNoisePSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(9, 9,
                11, 11,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getAccelerometerBiasPSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(12, 12,
                14, 14,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getGyroBiasPSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setElementAt(15, 15,
                config.getClockPhasePSD() * TIME_INTERVAL_SECONDS);
        qPrimeMatrix.setElementAt(16, 16,
                config.getClockFrequencyPSD() * TIME_INTERVAL_SECONDS);

        // 3. Propagate state estimates using (3.14) noting that only the clock
        // states are non-zero due to closed-loop correction.
        final Matrix xEstPropagated = new Matrix(17, 1);
        xEstPropagated.setElementAtIndex(15,
                previousState.getReceiverClockOffset()
                        + previousState.getReceiverClockDrift() * TIME_INTERVAL_SECONDS);
        xEstPropagated.setElementAtIndex(16,
                previousState.getReceiverClockDrift());

        // 4. Propagate state estimation error covariance matrix using (3.46)
        final Matrix pMatrixOld = previousState.getCovariance();
        final Matrix phiTrans = phiMatrix.transposeAndReturnNew();
        final Matrix halfQ = qPrimeMatrix.multiplyByScalarAndReturnNew(0.5);
        final Matrix tmp1 = pMatrixOld.addAndReturnNew(halfQ);
        final Matrix pMatrixPropagated = phiMatrix.multiplyAndReturnNew(tmp1);
        pMatrixPropagated.multiply(phiTrans);
        pMatrixPropagated.add(halfQ);

        // MEASUREMENT UPDATE PHASE

        final int noMeas = measurements.size();
        final Matrix uAseT = new Matrix(noMeas, 3);
        final Matrix predMeas = new Matrix(noMeas, 2);

        final double prevVx = previousState.getVx();
        final double prevVy = previousState.getVy();
        final double prevVz = previousState.getVz();
        final Matrix estVebeOld = Matrix.newFromArray(new double[]{prevVx, prevVy, prevVz});

        // Loop measurements
        for (int j = 0; j < noMeas; j++) {
            // Predict approx range
            final GNSSMeasurement measurement = measurements.get(j);
            Matrix deltaR = Matrix.newFromArray(new double[]{
                    measurement.getX() - prevX,
                    measurement.getY() - prevY,
                    measurement.getZ() - prevZ
            });

            final double approxRange = Math.sqrt(Utils.dotProduct(
                    deltaR.transposeAndReturnNew(), deltaR));

            // Calculate frame rotation during signal transit time using (8.36)
            final Matrix cei = new Matrix(3, 3);
            cei.fromArray(new double[]{
                    1.0, Constants.EARTH_ROTATION_RATE * approxRange / GNSSKalmanEpochEstimator.SPEED_OF_LIGHT, 0.0,
                    -Constants.EARTH_ROTATION_RATE * approxRange / GNSSKalmanEpochEstimator.SPEED_OF_LIGHT, 1.0, 0.0,
                    0.0, 0.0, 1.0}, false);

            // Predict pseudo-range using (9.165)
            final Matrix measurementPosition = Matrix.newFromArray(new double[]{
                    measurement.getX(), measurement.getY(), measurement.getZ()});
            final Matrix measurementVelocity = Matrix.newFromArray(new double[]{
                    measurement.getVx(), measurement.getVy(), measurement.getVz()});
            deltaR = cei.multiplyAndReturnNew(measurementPosition);
            deltaR.subtract(estRebeOld);
            final double range = Math.sqrt(Utils.dotProduct(
                    deltaR.transposeAndReturnNew(), deltaR));
            predMeas.setElementAt(j, 0,
                    range + xEstPropagated.getElementAtIndex(15));

            // Predict line of sight
            uAseT.setSubmatrix(j, 0, j, 2,
                    deltaR.transposeAndReturnNew().multiplyByScalarAndReturnNew(1.0 / range));

            // Predict pseudo-range rate using (9.165)
            final Matrix rangeRate = uAseT.getSubmatrix(j,
                    0, j, 2).multiplyAndReturnNew(
                    cei.multiplyAndReturnNew(measurementVelocity.addAndReturnNew(
                            omegaIe.multiplyAndReturnNew(measurementPosition)))
                            .subtractAndReturnNew(estVebeOld.addAndReturnNew(
                                    omegaIe.multiplyAndReturnNew(estRebeOld))));

            predMeas.setElementAt(j, 1,
                    rangeRate.getElementAtIndex(0)
                            + xEstPropagated.getElementAtIndex(16));
        }

        // 5. Set-up measurement matrix using (14.126)
        final Matrix hMatrix = new Matrix(2 * noMeas, 17);
        hMatrix.setSubmatrix(0, 6,
                noMeas - 1, 8,
                uAseT.getSubmatrix(0, 0,
                        noMeas - 1, 2));
        final double[] ones = new double[noMeas];
        Arrays.fill(ones, 1.0);
        hMatrix.setSubmatrix(0, 15,
                noMeas - 1, 15, ones);
        hMatrix.setSubmatrix(noMeas, 3,
                2 * noMeas - 1, 5,
                uAseT.getSubmatrix(0, 0,
                        noMeas - 1, 2));
        hMatrix.setSubmatrix(noMeas, 16,
                2 * noMeas - 1, 16, ones);

        // 6. Set-up measurement noise covariance matrix assuming all measurements
        // are independent and have equal variance for a given measurement type.
        final Matrix rMatrix = new Matrix(2 * noMeas, 2 * noMeas);
        rMatrix.setSubmatrix(0, 0,
                noMeas - 1, noMeas - 1,
                Matrix.identity(noMeas, noMeas).multiplyByScalarAndReturnNew(
                        Math.pow(config.getPseudoRangeSD(), 2.0)));
        rMatrix.setSubmatrix(0, noMeas,
                noMeas - 1, 2 * noMeas - 1,
                new Matrix(noMeas, noMeas));
        rMatrix.setSubmatrix(noMeas, 0,
                2 * noMeas - 1, noMeas - 1,
                new Matrix(noMeas, noMeas));
        rMatrix.setSubmatrix(noMeas, noMeas,
                2 * noMeas - 1, 2 * noMeas - 1,
                Matrix.identity(noMeas, noMeas).multiplyByScalarAndReturnNew(
                        Math.pow(config.getRangeRateSD(), 2.0)));

        // 7. Calculate Kalman gain using (3.21)
        final Matrix kMatrix = pMatrixPropagated.multiplyAndReturnNew(
                hMatrix.transposeAndReturnNew().multiplyAndReturnNew(
                        Utils.inverse(hMatrix.multiplyAndReturnNew(pMatrixPropagated)
                                .multiplyAndReturnNew(hMatrix.transposeAndReturnNew())
                                .addAndReturnNew(rMatrix))));

        // 8. Formulate measurement innovations using (14.119)
        final Matrix deltaZ = new Matrix(2 * noMeas, 1);
        for (int j = 0; j < noMeas; j++) {
            final GNSSMeasurement measurement = measurements.get(j);
            deltaZ.setElementAtIndex(j,
                    measurement.getPseudoRange() - predMeas.getElementAtIndex(j));
            deltaZ.setElementAtIndex(noMeas + j,
                    measurement.getPseudoRate()
                            - predMeas.getElementAtIndex(noMeas + j));
        }

        // 9. Update state estimates using (3.24)
        final Matrix xEstNew = xEstPropagated.addAndReturnNew(
                kMatrix.multiplyAndReturnNew(deltaZ));

        // 10. Update state estimation error covariance matrix using (3.25)
        final Matrix pMatrixNew = Matrix.identity(17, 17)
                .subtractAndReturnNew(kMatrix.multiplyAndReturnNew(hMatrix))
                .multiplyAndReturnNew(pMatrixPropagated);

        // CLOSED-LOOP CORRECTION

        // Correct attritude, velocity, and position using (14.7-9)
        final Matrix estCbeNew = Matrix.identity(3, 3)
                .subtractAndReturnNew(Utils.skewMatrix(
                        xEstNew.getSubmatrix(0, 0,
                                2, 0)));
        estCbeNew.multiply(estCbeOld);
        final Matrix estVebeNew = estVebeOld.subtractAndReturnNew(
                xEstNew.getSubmatrix(3, 0,
                        5, 0));
        final Matrix estRebeNew = estRebeOld.subtractAndReturnNew(
                xEstNew.getSubmatrix(6, 0,
                        8, 0));

        // Update IMU bias and GNSS receiver clock estimates
        final Matrix estIMUbiasOld = Matrix.newFromArray(new double[]{
                previousState.getAccelerationBiasX(),
                previousState.getAccelerationBiasY(),
                previousState.getAccelerationBiasZ(),
                previousState.getGyroBiasX(),
                previousState.getGyroBiasY(),
                previousState.getGyroBiasZ()
        });
        final Matrix estIMUbiasNew = estIMUbiasOld.addAndReturnNew(
                xEstNew.getSubmatrix(9, 0,
                        14, 0));

        final Matrix estClockNew = xEstNew.getSubmatrix(15, 0,
                16, 0);

        return new INSTightlyCoupledKalmanState(estCbeNew,
                estVebeNew.getElementAtIndex(0),
                estVebeNew.getElementAtIndex(1),
                estVebeNew.getElementAtIndex(2),
                estRebeNew.getElementAtIndex(0),
                estRebeNew.getElementAtIndex(1),
                estRebeNew.getElementAtIndex(2),
                estIMUbiasNew.getElementAtIndex(0),
                estIMUbiasNew.getElementAtIndex(1),
                estIMUbiasNew.getElementAtIndex(2),
                estIMUbiasNew.getElementAtIndex(3),
                estIMUbiasNew.getElementAtIndex(4),
                estIMUbiasNew.getElementAtIndex(5),
                estClockNew.getElementAtIndex(0),
                estClockNew.getElementAtIndex(1),
                pMatrixNew);
    }
}
