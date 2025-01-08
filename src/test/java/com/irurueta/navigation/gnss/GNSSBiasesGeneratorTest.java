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
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.exceptions.base.MockitoException;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
class GNSSBiasesGeneratorTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double MIN_USER_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_USER_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double MIN_SAT_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 150000.0;
    private static final double MAX_SAT_POSITION_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 800000.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testGenerateBias() throws WrongSizeException {
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final var randomizer = new UniformRandomizer();
        final var satellitePosition = new ECEFPosition(
                randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
        final var userPosition = new ECEFPosition(
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));
        final var config = generateConfig();

        final var bias1 = GNSSBiasesGenerator.generateBias(satellitePosition, userPosition, config, random);
        final var bias2 = generateBias(satellitePosition, userPosition, config, random);

        assertEquals(bias1, bias2, ABSOLUTE_ERROR);
    }

    @Test
    void testGenerateBiases() throws WrongSizeException {
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final var config = generateConfig();
        final var numSatellites = config.getNumberOfSatellites();

        final var randomizer = new UniformRandomizer(new Random());

        final var userPosition = new ECEFPosition(
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));

        final var satellitePositions = new ArrayList<ECEFPosition>();
        final var biases1 = new ArrayList<Double>();
        for (var i = 0; i < numSatellites; i++) {
            final var satellitePosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
            satellitePositions.add(satellitePosition);

            biases1.add(generateBias(satellitePosition, userPosition, config, random));
        }

        final var biases2 = new ArrayList<Double>();
        GNSSBiasesGenerator.generateBiases(satellitePositions, userPosition, config, random, biases2);
        final var biases3 = GNSSBiasesGenerator.generateBiases(
                satellitePositions, userPosition, config, random);

        assertEquals(biases2.size(), numSatellites);
        assertEquals(biases3.size(), numSatellites);
        for (var i = 0; i < numSatellites; i++) {
            assertEquals(biases1.get(i), biases2.get(i), ABSOLUTE_ERROR);
            assertEquals(biases1.get(i), biases3.get(i), ABSOLUTE_ERROR);
            assertEquals(biases2.get(i), biases3.get(i), 0.0);
        }
    }

    private static double generateBias(
            final ECEFPosition satellitePosition, final ECEFPosition userPosition, final GNSSConfig config,
            final Random random) throws WrongSizeException {
        final var userNedPosition = new NEDPosition();
        final var userNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(userPosition, new ECEFVelocity(), userNedPosition,
                userNedVelocity);
        final var userLatitude = userNedPosition.getLatitude();
        final var userLongitude = userNedPosition.getLongitude();
        final var cen = CoordinateTransformation.ecefToNedMatrix(userLatitude, userLongitude);

        final var satREsE = Matrix.newFromArray(satellitePosition.getPosition().asArray());
        final var rEaE = Matrix.newFromArray(userPosition.getPosition().asArray());
        final var deltaR = satREsE.subtractAndReturnNew(rEaE);
        final var norm = Utils.normF(deltaR);
        final var uAsE = deltaR.multiplyByScalarAndReturnNew(1.0 / norm);

        var elevation = -Math.asin(cen.getSubmatrix(2, 0, 2, 2)
                .multiplyAndReturnNew(uAsE).getElementAtIndex(0));

        elevation = Math.max(elevation, Math.toRadians(config.getMaskAngleDegrees()));

        final var ionoSD = config.getZenithIonosphereErrorSD()
                / Math.sqrt(1.0 - 0.899 * Math.pow(Math.cos(elevation), 2.0));
        final var tropSD = config.getZenithTroposphereErrorSD()
                / Math.sqrt(1.0 - 0.998 * Math.pow(Math.cos(elevation), 2.0));

        return config.getSISErrorSD() * random.nextGaussian() + ionoSD * random.nextGaussian()
                + tropSD * random.nextGaussian();
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
}
