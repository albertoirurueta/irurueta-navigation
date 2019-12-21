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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class GNSSMeasurementsGeneratorTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double MIN_USER_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_USER_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SAT_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 150000.0;
    private static final double MAX_SAT_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 800000.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_SAT_VELOCITY_VALUE = -100.0;
    private static final double MAX_SAT_VELOCITY_VALUE = 100.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final int MIN_NUM_SAT = 4;
    private static final int MAX_NUM_SAT = 10;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testGenerateSingleMeasurement() throws WrongSizeException {
        for (int t = 0; t < TIMES; t++) {
            final Random random = mock(Random.class);
            when(random.nextGaussian()).thenReturn(0.5);
            when(random.nextDouble()).thenReturn(0.5);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
            final Time time = new Time(timeSeconds, TimeUnit.SECOND);

            final ECEFPosition satellitePosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
            final ECEFPosition userPosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                    randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                    randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));

            final ECEFVelocity satelliteVelocity = new ECEFVelocity(
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE));
            final ECEFVelocity userVelocity = new ECEFVelocity(
                    randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE));

            final double satelliteX = satellitePosition.getX();
            final double satelliteY = satellitePosition.getY();
            final double satelliteZ = satellitePosition.getZ();

            final double satelliteVx = satelliteVelocity.getVx();
            final double satelliteVy = satelliteVelocity.getVy();
            final double satelliteVz = satelliteVelocity.getVz();

            final double userX = userPosition.getX();
            final double userY = userPosition.getY();
            final double userZ = userPosition.getZ();

            final double userVx = userVelocity.getVx();
            final double userVy = userVelocity.getVy();
            final double userVz = userVelocity.getVz();

            final ECEFPositionAndVelocity satellitePositionAndVelocity =
                    new ECEFPositionAndVelocity(satellitePosition, satelliteVelocity);
            final ECEFPositionAndVelocity userPositionAndVelocity =
                    new ECEFPositionAndVelocity(userPosition, userVelocity);

            final GNSSConfig config = generateConfig();

            final double bias = GNSSBiasesGenerator.generateBias(satellitePosition,
                    userPosition, config, random);

            final GNSSMeasurement measurement1 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(timeSeconds,
                    satelliteX, satelliteY, satelliteZ,
                    satelliteVx, satelliteVy, satelliteVz,
                    userX, userY, userZ,
                    userVx, userVy, userVz, bias, config, random, measurement1));

            final GNSSMeasurement measurement2 = GNSSMeasurementsGenerator
                    .generate(timeSeconds, satelliteX, satelliteY, satelliteZ,
                            satelliteVx, satelliteVy, satelliteVz,
                            userX, userY, userZ,
                            userVx, userVy, userVz, bias, config, random);
            assertNotNull(measurement2);

            final GNSSMeasurement measurement3 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePosition, satelliteVelocity, userPosition,
                    userVelocity, bias, config, random, measurement3));

            final GNSSMeasurement measurement4 = GNSSMeasurementsGenerator
                    .generate(timeSeconds, satellitePosition, satelliteVelocity,
                            userPosition, userVelocity, bias, config, random);
            assertNotNull(measurement4);

            final GNSSMeasurement measurement5 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(time, satellitePosition,
                    satelliteVelocity, userPosition, userVelocity, bias,
                    config, random, measurement5));

            final GNSSMeasurement measurement6 = GNSSMeasurementsGenerator
                    .generate(time, satellitePosition, satelliteVelocity,
                            userPosition, userVelocity, bias, config, random);
            assertNotNull(measurement6);

            final GNSSMeasurement measurement7 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionAndVelocity, userPositionAndVelocity,
                    bias, config, random, measurement7));

            final GNSSMeasurement measurement8 = GNSSMeasurementsGenerator
                    .generate(timeSeconds, satellitePositionAndVelocity,
                            userPositionAndVelocity, bias, config, random);
            assertNotNull(measurement8);

            final GNSSMeasurement measurement9 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(time, satellitePositionAndVelocity,
                    userPositionAndVelocity, bias, config, random, measurement9));

            final GNSSMeasurement measurement10 = GNSSMeasurementsGenerator
                    .generate(time, satellitePositionAndVelocity,
                            userPositionAndVelocity, bias, config, random);
            assertNotNull(measurement10);

            assertEquals(measurement1, measurement2);
            assertEquals(measurement1, measurement3);
            assertEquals(measurement1, measurement4);
            assertEquals(measurement1, measurement5);
            assertEquals(measurement1, measurement6);
            assertEquals(measurement1, measurement7);
            assertEquals(measurement1, measurement8);
            assertEquals(measurement1, measurement9);
            assertEquals(measurement1, measurement10);

            final GNSSMeasurement measurement = generate(timeSeconds,
                    satellitePositionAndVelocity, userPositionAndVelocity,
                    bias, config, random);

            assertNotNull(measurement);
            assertTrue(measurement.equals(measurement1, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testGenerateMultipleMeasurements() throws WrongSizeException {
        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);
        when(random.nextDouble()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final int numSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);

        final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final Time time = new Time(timeSeconds, TimeUnit.SECOND);

        final ECEFPosition userPosition = new ECEFPosition(
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));
        final ECEFVelocity userVelocity = new ECEFVelocity(
                randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE));

        final double userX = userPosition.getX();
        final double userY = userPosition.getY();
        final double userZ = userPosition.getZ();

        final double userVx = userVelocity.getVx();
        final double userVy = userVelocity.getVy();
        final double userVz = userVelocity.getVz();

        final ECEFPositionAndVelocity userPositionAndVelocity =
                new ECEFPositionAndVelocity(userPosition, userVelocity);

        final GNSSConfig config = generateConfig();

        final List<Double> biases = new ArrayList<>();
        final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities =
                new ArrayList<>();
        final List<GNSSMeasurement> expectedResult = new ArrayList<>();
        for (int n = 0; n < numSatellites; n++) {
            final ECEFPosition satellitePosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
            final ECEFVelocity satelliteVelocity = new ECEFVelocity(
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE));

            final ECEFPositionAndVelocity satellitePositionAndVelocity =
                    new ECEFPositionAndVelocity(satellitePosition, satelliteVelocity);

            final double bias = GNSSBiasesGenerator.generateBias(satellitePosition,
                    userPosition, config, random);

            biases.add(bias);
            satellitePositionsAndVelocities.add(satellitePositionAndVelocity);

            final GNSSMeasurement measurement = generate(timeSeconds,
                    satellitePositionAndVelocity, userPositionAndVelocity,
                    bias, config, random);

            expectedResult.add(measurement);
        }

        final List<GNSSMeasurement> result1 = new ArrayList<>();
        GNSSMeasurementsGenerator.generate(timeSeconds,
                satellitePositionsAndVelocities, userX, userY, userZ,
                userVx, userVy, userVz, biases, config, random, result1);

        final Collection<GNSSMeasurement> result2 =
                GNSSMeasurementsGenerator.generate(timeSeconds,
                        satellitePositionsAndVelocities, userX, userY, userZ,
                        userVx, userVy, userVz, biases, config, random);

        final List<GNSSMeasurement> result3 = new ArrayList<>();
        GNSSMeasurementsGenerator.generate(timeSeconds,
                satellitePositionsAndVelocities, userPosition, userVelocity,
                biases, config, random, result3);

        final Collection<GNSSMeasurement> result4 =
                GNSSMeasurementsGenerator.generate(timeSeconds,
                        satellitePositionsAndVelocities, userPosition, userVelocity,
                        biases, config, random);

        final List<GNSSMeasurement> result5 = new ArrayList<>();
        GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities,
                userPosition, userVelocity, biases, config, random, result5);

        final Collection<GNSSMeasurement> result6 =
                GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities,
                        userPosition, userVelocity, biases, config, random);

        final List<GNSSMeasurement> result7 = new ArrayList<>();
        GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities,
                userPositionAndVelocity, biases, config, random, result7);

        final Collection<GNSSMeasurement> result8 =
                GNSSMeasurementsGenerator.generate(timeSeconds,
                        satellitePositionsAndVelocities, userPositionAndVelocity,
                        biases, config, random);

        final List<GNSSMeasurement> result9 = new ArrayList<>();
        GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities,
                userPositionAndVelocity, biases, config, random, result9);

        final Collection<GNSSMeasurement> result10 =
                GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities,
                        userPositionAndVelocity, biases, config, random);

        assertEquals(result1.size(), numSatellites);
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);

        assertEquals(expectedResult.size(), numSatellites);
        for (int i = 0; i < numSatellites; i++) {
            final GNSSMeasurement m1 = expectedResult.get(i);
            final GNSSMeasurement m2 = result1.get(i);

            assertTrue(m1.equals(m2, ABSOLUTE_ERROR));
        }
    }

    private static GNSSConfig generateConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        return new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
    }

    private static GNSSMeasurement generate(
            final double time, final ECEFPositionAndVelocity satPosAndVel,
            final ECEFPositionAndVelocity userPosAndVel, final double bias,
            final GNSSConfig config, final Random random) throws WrongSizeException {

        final ECEFPosition satPosition = satPosAndVel.getEcefPosition();
        final ECEFVelocity satVelocity = satPosAndVel.getEcefVelocity();

        final ECEFPosition userPosition = userPosAndVel.getEcefPosition();
        final ECEFVelocity userVelocity = userPosAndVel.getEcefVelocity();

        final NEDPosition userNedPosition = new NEDPosition();
        final NEDVelocity userNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(userPosition, userVelocity,
                userNedPosition, userNedVelocity);

        final Matrix cen = CoordinateTransformation.ecefToNedMatrix(
                userNedPosition.getLatitude(), userNedPosition.getLongitude());

        final Matrix omegaIe = Utils.skewMatrix(
                new double[]{0.0, 0.0, Constants.EARTH_ROTATION_RATE});

        final Matrix satRese = satPosition.asMatrix();
        final Matrix rEae = userPosition.asMatrix();
        Matrix deltaR = satRese.subtractAndReturnNew(rEae);
        final double approxRange = Utils.normF(deltaR);
        final Matrix uase = deltaR.multiplyByScalarAndReturnNew(1.0 / approxRange);

        final double elevation = -Math.asin(
                cen.getSubmatrix(2, 0, 2, 2)
                        .multiplyAndReturnNew(uase).getElementAtIndex(0));

        if (elevation >= Math.toRadians(config.getMaskAngleDegrees())) {
            final double value = Constants.EARTH_ROTATION_RATE * approxRange / Constants.SPEED_OF_LIGHT;
            final Matrix cei = new Matrix(3, 3);
            cei.setSubmatrix(0, 0, 2, 2,
                    new double[]{1.0, -value, 0.0, value, 1.0, 0.0, 0.0, 0.0, 1.0});

            deltaR = cei.multiplyAndReturnNew(satRese).subtractAndReturnNew(rEae);
            final double range = Utils.normF(deltaR);

            final Matrix satvese = satVelocity.asMatrix();
            final Matrix veae = userVelocity.asMatrix();

            final double rangeRate = uase.transposeAndReturnNew()
                    .multiplyAndReturnNew(cei.multiplyAndReturnNew(
                            satvese.addAndReturnNew(omegaIe.multiplyAndReturnNew(satRese)))
                            .subtractAndReturnNew(veae.addAndReturnNew(omegaIe.multiplyAndReturnNew(rEae))))
                    .getElementAtIndex(0);

            final double pseudoRange = range + bias
                    + config.getInitialReceiverClockOffset()
                    + config.getInitialReceiverClockDrift() * time
                    + config.getCodeTrackingErrorSD() * random.nextGaussian();

            final double pseudoRangeRate = rangeRate
                    + config.getInitialReceiverClockDrift()
                    + config.getRangeRateTrackingErrorSD() * random.nextGaussian();

            return new GNSSMeasurement(pseudoRange, pseudoRangeRate, satPosAndVel);
        } else {
            return null;
        }
    }
}
