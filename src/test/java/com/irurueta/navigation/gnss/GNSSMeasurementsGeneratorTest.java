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
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
class GNSSMeasurementsGeneratorTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double MIN_USER_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_USER_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SAT_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 150000.0;
    private static final double MAX_SAT_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 800000.0;

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
    void testGenerateSingleMeasurement() throws WrongSizeException {
        for (var t = 0; t < TIMES; t++) {
            final var random = mock(Random.class);
            when(random.nextGaussian()).thenReturn(0.5);

            final var randomizer = new UniformRandomizer();

            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
            final var time = new Time(timeSeconds, TimeUnit.SECOND);

            final var satellitePosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
            final var userPosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                    randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                    randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));

            final var satelliteVelocity = new ECEFVelocity(
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE));
            final var userVelocity = new ECEFVelocity(
                    randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE));

            final var satelliteX = satellitePosition.getX();
            final var satelliteY = satellitePosition.getY();
            final var satelliteZ = satellitePosition.getZ();

            final var satelliteVx = satelliteVelocity.getVx();
            final var satelliteVy = satelliteVelocity.getVy();
            final var satelliteVz = satelliteVelocity.getVz();

            final var userX = userPosition.getX();
            final var userY = userPosition.getY();
            final var userZ = userPosition.getZ();

            final var userVx = userVelocity.getVx();
            final var userVy = userVelocity.getVy();
            final var userVz = userVelocity.getVz();

            final var satellitePositionAndVelocity = new ECEFPositionAndVelocity(satellitePosition, satelliteVelocity);
            final var userPositionAndVelocity = new ECEFPositionAndVelocity(userPosition, userVelocity);

            final var config = generateConfig();

            final var bias = GNSSBiasesGenerator.generateBias(satellitePosition, userPosition, config, random);

            final var measurement1 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(timeSeconds, satelliteX, satelliteY, satelliteZ,
                    satelliteVx, satelliteVy, satelliteVz, userX, userY, userZ, userVx, userVy, userVz, bias, config,
                    random, measurement1));

            final var measurement2 = GNSSMeasurementsGenerator.generate(timeSeconds, satelliteX, satelliteY, satelliteZ,
                    satelliteVx, satelliteVy, satelliteVz, userX, userY, userZ, userVx, userVy, userVz, bias, config,
                    random);
            assertNotNull(measurement2);

            final var measurement3 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(timeSeconds, satellitePosition, satelliteVelocity,
                    userPosition, userVelocity, bias, config, random, measurement3));

            final var measurement4 = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePosition,
                    satelliteVelocity, userPosition, userVelocity, bias, config, random);
            assertNotNull(measurement4);

            final var measurement5 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(time, satellitePosition, satelliteVelocity, userPosition,
                    userVelocity, bias, config, random, measurement5));

            final var measurement6 = GNSSMeasurementsGenerator.generate(time, satellitePosition, satelliteVelocity,
                    userPosition, userVelocity, bias, config, random);
            assertNotNull(measurement6);

            final var measurement7 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionAndVelocity,
                    userPositionAndVelocity, bias, config, random, measurement7));

            final var measurement8 = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionAndVelocity,
                    userPositionAndVelocity, bias, config, random);
            assertNotNull(measurement8);

            final var measurement9 = new GNSSMeasurement();
            assertTrue(GNSSMeasurementsGenerator.generate(time, satellitePositionAndVelocity, userPositionAndVelocity,
                    bias, config, random, measurement9));

            final var measurement10 = GNSSMeasurementsGenerator.generate(time, satellitePositionAndVelocity,
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

            final var measurement = generate(timeSeconds, satellitePositionAndVelocity, userPositionAndVelocity, bias,
                    config, random);

            assertNotNull(measurement);
            assertTrue(measurement.equals(measurement1, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testGenerateMultipleMeasurements() throws WrongSizeException {
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final var randomizer = new UniformRandomizer(new Random());

        final var numSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);

        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);
        final var time = new Time(timeSeconds, TimeUnit.SECOND);

        final var userPosition = new ECEFPosition(
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));
        final var userVelocity = new ECEFVelocity(
                randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE),
                randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE));

        final var userX = userPosition.getX();
        final var userY = userPosition.getY();
        final var userZ = userPosition.getZ();

        final var userVx = userVelocity.getVx();
        final var userVy = userVelocity.getVy();
        final var userVz = userVelocity.getVz();

        final var userPositionAndVelocity = new ECEFPositionAndVelocity(userPosition, userVelocity);

        final var config = generateConfig();

        final var biases = new ArrayList<Double>();
        final var satellitePositionsAndVelocities = new ArrayList<ECEFPositionAndVelocity>();
        final var expectedResult = new ArrayList<GNSSMeasurement>();
        for (var n = 0; n < numSatellites; n++) {
            final var satellitePosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
            final var satelliteVelocity = new ECEFVelocity(
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE),
                    randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE));

            final var satellitePositionAndVelocity = new ECEFPositionAndVelocity(satellitePosition, satelliteVelocity);

            final var bias = GNSSBiasesGenerator.generateBias(satellitePosition, userPosition, config, random);

            biases.add(bias);
            satellitePositionsAndVelocities.add(satellitePositionAndVelocity);

            final var measurement = generate(timeSeconds, satellitePositionAndVelocity, userPositionAndVelocity, bias,
                    config, random);

            expectedResult.add(measurement);
        }

        final var result1 = new ArrayList<GNSSMeasurement>();
        GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities, userX, userY, userZ, userVx,
                userVy, userVz, biases, config, random, result1);

        final var result2 = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities,
                userX, userY, userZ, userVx, userVy, userVz, biases, config, random);

        final var result3 = new ArrayList<GNSSMeasurement>();
        GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities, userPosition, userVelocity,
                biases, config, random, result3);

        final var result4 = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities,
                userPosition, userVelocity, biases, config, random);

        final var result5 = new ArrayList<GNSSMeasurement>();
        GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities, userPosition, userVelocity, biases,
                config, random, result5);

        final var result6 = GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities, userPosition,
                userVelocity, biases, config, random);

        final var result7 = new ArrayList<GNSSMeasurement>();
        GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities, userPositionAndVelocity,
                biases, config, random, result7);

        final var result8 = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities,
                userPositionAndVelocity, biases, config, random);

        final var result9 = new ArrayList<GNSSMeasurement>();
        GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities, userPositionAndVelocity, biases,
                config, random, result9);

        final var result10 = GNSSMeasurementsGenerator.generate(time, satellitePositionsAndVelocities,
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
        for (var i = 0; i < numSatellites; i++) {
            final var m1 = expectedResult.get(i);
            final var m2 = result1.get(i);

            assertTrue(m1.equals(m2, ABSOLUTE_ERROR));
        }
    }

    private static GNSSConfig generateConfig() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
    }

    private static GNSSMeasurement generate(
            final double time, final ECEFPositionAndVelocity satPosAndVel, final ECEFPositionAndVelocity userPosAndVel,
            final double bias, final GNSSConfig config, final Random random) throws WrongSizeException {

        final var satPosition = satPosAndVel.getEcefPosition();
        final var satVelocity = satPosAndVel.getEcefVelocity();

        final var userPosition = userPosAndVel.getEcefPosition();
        final var userVelocity = userPosAndVel.getEcefVelocity();

        final var userNedPosition = new NEDPosition();
        final var userNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(userPosition, userVelocity, userNedPosition,
                userNedVelocity);

        final var cen = CoordinateTransformation.ecefToNedMatrix(userNedPosition.getLatitude(),
                userNedPosition.getLongitude());

        final var omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, Constants.EARTH_ROTATION_RATE});

        final var satRese = satPosition.asMatrix();
        final var rEae = userPosition.asMatrix();
        var deltaR = satRese.subtractAndReturnNew(rEae);
        final var approxRange = Utils.normF(deltaR);
        final var uase = deltaR.multiplyByScalarAndReturnNew(1.0 / approxRange);

        final var elevation = -Math.asin(
                cen.getSubmatrix(2, 0, 2, 2)
                        .multiplyAndReturnNew(uase).getElementAtIndex(0));

        if (elevation >= Math.toRadians(config.getMaskAngleDegrees())) {
            final var value = Constants.EARTH_ROTATION_RATE * approxRange / Constants.SPEED_OF_LIGHT;
            final var cei = new Matrix(3, 3);
            cei.setSubmatrix(0, 0, 2, 2,
                    new double[]{1.0, -value, 0.0, value, 1.0, 0.0, 0.0, 0.0, 1.0});

            deltaR = cei.multiplyAndReturnNew(satRese).subtractAndReturnNew(rEae);
            final var range = Utils.normF(deltaR);

            final var satvese = satVelocity.asMatrix();
            final var veae = userVelocity.asMatrix();

            final var rangeRate = uase.transposeAndReturnNew().multiplyAndReturnNew(cei.multiplyAndReturnNew(
                    satvese.addAndReturnNew(omegaIe.multiplyAndReturnNew(satRese))).subtractAndReturnNew(
                            veae.addAndReturnNew(omegaIe.multiplyAndReturnNew(rEae)))).getElementAtIndex(0);

            final var pseudoRange = range + bias + config.getInitialReceiverClockOffset()
                    + config.getInitialReceiverClockDrift() * time
                    + config.getCodeTrackingErrorSD() * random.nextGaussian();

            final var pseudoRangeRate = rangeRate + config.getInitialReceiverClockDrift()
                    + config.getRangeRateTrackingErrorSD() * random.nextGaussian();

            return new GNSSMeasurement(pseudoRange, pseudoRangeRate, satPosAndVel);
        } else {
            return null;
        }
    }
}
