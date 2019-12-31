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
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class GNSSKalmanEpochEstimatorTest {

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
    private static final double CLOCK_OFFSET = 0.5;
    private static final double CLOCK_DRIFT = 1e-4;

    private static final double MIN_CONFIG_VALUE = 1e-4;
    private static final double MAX_CONFIG_VALUE = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    public void testEstimate() throws AlgebraException {
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

            final GNSSEstimation previousEstimation = new GNSSEstimation(
                    userEcefFrame.getX(), userEcefFrame.getY(), userEcefFrame.getZ(),
                    userEcefFrame.getVx(), userEcefFrame.getVy(), userEcefFrame.getVz(),
                    CLOCK_OFFSET, CLOCK_DRIFT);

            final Matrix previousCovariance = Matrix.identity(
                    GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);

            final double initialPositionUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double initialVelocityUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double initialClockOffsetUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double initialClockDriftUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double accelerationPSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double clockFrequencyPSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double clockPhasePSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double pseudoRangeSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double rangeRateSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);

            final GNSSKalmanConfig config = new GNSSKalmanConfig(
                    initialPositionUncertainty, initialVelocityUncertainty,
                    initialClockOffsetUncertainty, initialClockDriftUncertainty,
                    accelerationPSD, clockFrequencyPSD,
                    clockPhasePSD, pseudoRangeSD, rangeRateSD);

            final GNSSEstimation updatedEstimation1 = new GNSSEstimation();
            final Matrix updatedCovariance1 = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                    GNSSEstimation.NUM_PARAMETERS);

            GNSSKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousEstimation, previousCovariance, config, updatedEstimation1,
                    updatedCovariance1);

            final GNSSEstimation updatedEstimation2 = new GNSSEstimation();
            final Matrix updatedCovariance2 = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                    GNSSEstimation.NUM_PARAMETERS);

            final Time propagationInterval = new Time(TIME_INTERVAL_SECONDS,
                    TimeUnit.SECOND);

            GNSSKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousEstimation, previousCovariance, config, updatedEstimation2,
                    updatedCovariance2);


            final GNSSEstimation updatedEstimation3 = new GNSSEstimation();
            final Matrix updatedCovariance3 = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                    GNSSEstimation.NUM_PARAMETERS);
            estimate(measurements, previousEstimation,
                    previousCovariance, config, updatedEstimation3, updatedCovariance3);

            assertNotNull(updatedEstimation1);

            assertEquals(updatedEstimation1.getX(), updatedEstimation3.getX(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation1.getY(), updatedEstimation3.getY(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation1.getZ(), updatedEstimation3.getZ(), ABSOLUTE_ERROR);

            assertEquals(updatedEstimation1.getVx(), updatedEstimation3.getVx(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation1.getVy(), updatedEstimation3.getVy(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation1.getVz(), updatedEstimation3.getVz(), ABSOLUTE_ERROR);

            assertEquals(updatedEstimation1.getClockOffset(),
                    updatedEstimation3.getClockOffset(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation1.getClockDrift(),
                    updatedEstimation3.getClockDrift(), ABSOLUTE_ERROR);

            assertTrue(updatedCovariance1.equals(updatedCovariance3, ABSOLUTE_ERROR));

            assertEquals(updatedEstimation1, updatedEstimation2);
            assertEquals(updatedCovariance1, updatedCovariance2);
        }
    }

    @Test
    public void testEstimateWithKalmanState() throws AlgebraException {
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

            final GNSSEstimation previousEstimation = new GNSSEstimation(
                    userEcefFrame.getX(), userEcefFrame.getY(), userEcefFrame.getZ(),
                    userEcefFrame.getVx(), userEcefFrame.getVy(), userEcefFrame.getVz(),
                    CLOCK_OFFSET, CLOCK_DRIFT);

            final Matrix previousCovariance = Matrix.identity(
                    GNSSEstimation.NUM_PARAMETERS, GNSSEstimation.NUM_PARAMETERS);

            final GNSSKalmanState previousState =
                    new GNSSKalmanState(previousEstimation, previousCovariance);

            final double initialPositionUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double initialVelocityUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double initialClockOffsetUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double initialClockDriftUncertainty = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double accelerationPSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double clockFrequencyPSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double clockPhasePSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double pseudoRangeSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);
            final double rangeRateSD = randomizer.nextDouble(
                    MIN_CONFIG_VALUE, MAX_CONFIG_VALUE);

            final GNSSKalmanConfig config = new GNSSKalmanConfig(
                    initialPositionUncertainty, initialVelocityUncertainty,
                    initialClockOffsetUncertainty, initialClockDriftUncertainty,
                    accelerationPSD, clockFrequencyPSD,
                    clockPhasePSD, pseudoRangeSD, rangeRateSD);

            final GNSSKalmanState updatedState1 = new GNSSKalmanState();

            GNSSKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, config, updatedState1);

            final Time propagationInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

            final GNSSKalmanState updatedState2 =
                    GNSSKalmanEpochEstimator.estimate(measurements, propagationInterval,
                            previousState, config);

            final GNSSKalmanState updatedState3 = new GNSSKalmanState();
            GNSSKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, config, updatedState3);

            final GNSSKalmanState updatedState4 =
                    GNSSKalmanEpochEstimator.estimate(measurements,
                            TIME_INTERVAL_SECONDS, previousState, config);

            final GNSSEstimation updatedEstimation = updatedState1.getEstimation();
            final Matrix updatedCovariance = updatedState1.getCovariance();

            final GNSSEstimation updatedEstimation2 = new GNSSEstimation();
            final Matrix updatedCovariance2 = new Matrix(GNSSEstimation.NUM_PARAMETERS,
                    GNSSEstimation.NUM_PARAMETERS);
            estimate(measurements, previousEstimation,
                    previousCovariance, config, updatedEstimation2, updatedCovariance2);

            assertNotNull(updatedEstimation);

            assertEquals(updatedEstimation.getX(), updatedEstimation2.getX(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation.getY(), updatedEstimation2.getY(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation.getZ(), updatedEstimation2.getZ(), ABSOLUTE_ERROR);

            assertEquals(updatedEstimation.getVx(), updatedEstimation2.getVx(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation.getVy(), updatedEstimation2.getVy(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation.getVz(), updatedEstimation2.getVz(), ABSOLUTE_ERROR);

            assertEquals(updatedEstimation.getClockOffset(),
                    updatedEstimation2.getClockOffset(), ABSOLUTE_ERROR);
            assertEquals(updatedEstimation.getClockDrift(),
                    updatedEstimation2.getClockDrift(), ABSOLUTE_ERROR);

            assertTrue(updatedCovariance.equals(updatedCovariance2, ABSOLUTE_ERROR));

            assertEquals(updatedState1, updatedState2);
            assertEquals(updatedState1, updatedState3);
            assertEquals(updatedState1, updatedState4);
        }
    }

    private void estimate(final List<GNSSMeasurement> measurements,
                          final GNSSEstimation previousEstimation,
                          final Matrix previousCovariance,
                          final GNSSKalmanConfig config,
                          final GNSSEstimation updatedEstimation,
                          final Matrix updatedCovariance) throws AlgebraException {

        // 1. Determine transition matrix using (9.147) and (9.150)
        final Matrix phiMatrix = Matrix.identity(8, 8);
        phiMatrix.setElementAt(0, 3, GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS);
        phiMatrix.setElementAt(1, 4, GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS);
        phiMatrix.setElementAt(2, 5, GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS);
        phiMatrix.setElementAt(6, 7, GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS);

        // 2. Determine system noise covariance matrix using (9.152)
        final Matrix qMatrix = new Matrix(8, 8);
        qMatrix.setSubmatrix(0, 0,
                2, 2,
                Matrix.identity(3, 3)
                        .multiplyByScalarAndReturnNew(config.getAccelerationPSD()
                                * Math.pow(GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS, 3.0) / 3.0));
        qMatrix.setSubmatrix(0, 3,
                2, 5,
                Matrix.identity(3, 3)
                        .multiplyByScalarAndReturnNew(config.getAccelerationPSD()
                                * Math.pow(GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS, 2.0) / 2.0));
        qMatrix.setSubmatrix(3, 0,
                5, 2,
                Matrix.identity(3, 3)
                        .multiplyByScalarAndReturnNew(config.getAccelerationPSD()
                                * Math.pow(GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS, 2.0) / 2.0));
        qMatrix.setSubmatrix(3, 3,
                5, 5,
                Matrix.identity(3, 3)
                        .multiplyByScalarAndReturnNew(config.getAccelerationPSD()
                                * GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS));
        qMatrix.setElementAt(6, 6, config.getClockFrequencyPSD()
                * Math.pow(GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS, 3.0) / 3.0
                + config.getClockPhasePSD() * GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS);
        qMatrix.setElementAt(6, 7, config.getClockFrequencyPSD()
                * Math.pow(GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS, 2.0) / 2.0);
        qMatrix.setElementAt(7, 6, config.getClockFrequencyPSD()
                * Math.pow(GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS, 2.0) / 2.0);
        qMatrix.setElementAt(7, 7, config.getClockFrequencyPSD()
                * GNSSKalmanEpochEstimatorTest.TIME_INTERVAL_SECONDS);

        // 3. Propagate state estimates using (3.14)
        final Matrix xEstOld = previousEstimation.asMatrix();
        final Matrix xEstPropagated = phiMatrix.multiplyAndReturnNew(xEstOld);

        // 4. Propagate state estimation error covariance matrix using (3.15)
        final Matrix pMatrixPropagated = phiMatrix.multiplyAndReturnNew(
                previousCovariance.multiplyAndReturnNew(
                        phiMatrix.transposeAndReturnNew()))
                .addAndReturnNew(qMatrix);

        // Measurement update phase

        // Skew symmetric matrix of Earth rate
        final Matrix omegaIe = Utils.skewMatrix(
                new double[]{0.0, 0.0, Constants.EARTH_ROTATION_RATE});

        final int noMeas = measurements.size();
        final Matrix uAseT = new Matrix(noMeas, 3);
        final Matrix predMeas = new Matrix(noMeas, 2);

        // Loop measurements
        for (int j = 0; j < noMeas; j++) {
            // Predict approx range
            final GNSSMeasurement measurement = measurements.get(j);
            Matrix deltaR = Matrix.newFromArray(new double[]{
                    measurement.getX() - xEstPropagated.getElementAtIndex(0),
                    measurement.getY() - xEstPropagated.getElementAtIndex(1),
                    measurement.getZ() - xEstPropagated.getElementAtIndex(2)
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
            final Matrix propagatedPosition = xEstPropagated.getSubmatrix(
                    0, 0, 2, 0);
            deltaR = cei.multiplyAndReturnNew(measurementPosition)
                    .subtractAndReturnNew(propagatedPosition);
            final double range = Math.sqrt(Utils.dotProduct(
                    deltaR.transposeAndReturnNew(), deltaR));
            predMeas.setElementAt(j, 0,
                    range + xEstPropagated.getElementAtIndex(6));

            // Predict line of sight
            uAseT.setSubmatrix(j, 0, j, 2,
                    deltaR.transposeAndReturnNew().multiplyByScalarAndReturnNew(1.0 / range));

            // Predict pseudo-range rate using (9.165)
            final Matrix rangeRate = uAseT.getSubmatrix(j,
                    0, j, 2).multiplyAndReturnNew(
                    cei.multiplyAndReturnNew(measurementVelocity.addAndReturnNew(
                            omegaIe.multiplyAndReturnNew(measurementPosition)))
                            .subtractAndReturnNew(xEstPropagated.getSubmatrix(
                                    3, 0, 5,
                                    0)
                                    .addAndReturnNew(omegaIe.multiplyAndReturnNew(
                                            xEstPropagated.getSubmatrix(
                                                    0, 0,
                                                    2,
                                                    0)))));
            predMeas.setElementAt(j, 1,
                    rangeRate.getElementAtIndex(0)
                            + xEstPropagated.getElementAtIndex(7));
        }

        // 5. Set-up measurement matrix using (9.163)
        final Matrix hMatrix = new Matrix(2 * noMeas, 8);
        hMatrix.setSubmatrix(0, 0,
                noMeas - 1, 2,
                uAseT.getSubmatrix(0, 0,
                        noMeas - 1, 2)
                        .multiplyByScalarAndReturnNew(-1.0));
        hMatrix.setSubmatrix(0, 3,
                noMeas - 1, 5,
                new Matrix(noMeas, 3));
        final double[] ones = new double[noMeas];
        Arrays.fill(ones, 1.0);
        hMatrix.setSubmatrix(0, 6,
                noMeas - 1, 6,
                ones);
        hMatrix.setSubmatrix(0, 7,
                noMeas - 1, 7,
                new double[noMeas]);
        hMatrix.setSubmatrix(noMeas, 0,
                2 * noMeas - 1, 2,
                new Matrix(noMeas, 3));
        hMatrix.setSubmatrix(noMeas, 3,
                2 * noMeas - 1, 5,
                uAseT.getSubmatrix(0, 0,
                        noMeas - 1, 2)
                        .multiplyByScalarAndReturnNew(-1.0));
        hMatrix.setSubmatrix(noMeas, 6,
                2 * noMeas - 1, 6,
                new Matrix(noMeas, 1));
        hMatrix.setSubmatrix(noMeas, 7,
                2 * noMeas - 1, 7, ones);

        // 6. Set-up measurement noise covariance matrix assuming all measurements
        // are independent and have equal variance for a given measurement type
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

        // 8. Formulate measurement innovations using (3.88)
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
        final Matrix pMatrixNew = Matrix.identity(8, 8)
                .subtractAndReturnNew(kMatrix.multiplyAndReturnNew(hMatrix))
                .multiplyAndReturnNew(pMatrixPropagated);

        updatedEstimation.fromMatrix(xEstNew);
        updatedCovariance.copyFrom(pMatrixNew);
    }
}
