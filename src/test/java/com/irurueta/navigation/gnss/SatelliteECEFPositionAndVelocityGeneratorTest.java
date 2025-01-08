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

import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SatelliteECEFPositionAndVelocityGeneratorTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double MIN_TIME_SECONDS = 0.0;
    private static final double MAX_TIME_SECONDS = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(SatelliteECEFPositionAndVelocityGenerator.EARTH_GRAVITATIONAL_CONSTANT,
                Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(SatelliteECEFPositionAndVelocityGenerator.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE,
                0.0);
    }

    @Test
    void testGenerateSatellitesPositionAndVelocity() {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
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

            final var config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                    orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                    zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                    initialReceiverClockDrift);

            final var time = randomizer.nextDouble(MIN_TIME_SECONDS, MAX_TIME_SECONDS);

            final var result1 = new ArrayList<ECEFPositionAndVelocity>();
            for (var j = 0; j < numberOfSatellites; j++) {
                result1.add(computeSatellitePositionAndVelocity(time, config, j));
            }

            final var result2 = SatelliteECEFPositionAndVelocityGenerator.generateSatellitesPositionAndVelocity(time,
                    config);
            final var result3 = new ArrayList<ECEFPositionAndVelocity>();
            SatelliteECEFPositionAndVelocityGenerator.generateSatellitesPositionAndVelocity(time, config, result3);

            var failed = false;
            var j = 0;
            for (final var posVel2 : result2) {
                final var posVel1 = result1.get(j);

                if (!posVel1.equals(posVel2, VERY_LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(posVel1.equals(posVel2, VERY_LARGE_ABSOLUTE_ERROR));
                j++;
            }

            if (failed) {
                continue;
            }

            assertEquals(result2, result3);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGenerateSatellitePositionAndVelocity() {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
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

            final var config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                    orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                    zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                    initialReceiverClockDrift);

            final var time = randomizer.nextDouble(MIN_TIME_SECONDS, MAX_TIME_SECONDS);

            var failed = false;
            for (var j = 0; j < numberOfSatellites; j++) {
                final var result1 = new ECEFPositionAndVelocity();
                SatelliteECEFPositionAndVelocityGenerator.generateSatellitePositionAndVelocity(time, config, j,
                        result1);
                final var result2 = SatelliteECEFPositionAndVelocityGenerator.generateSatellitePositionAndVelocity(time,
                        config, j);

                final var result3 = computeSatellitePositionAndVelocity(time, config, j);

                if (!result1.equals(result3, LARGE_ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                if (!result2.equals(result3, ABSOLUTE_ERROR)) {
                    failed = true;
                    break;
                }
                assertTrue(result1.equals(result3, LARGE_ABSOLUTE_ERROR));
                assertTrue(result2.equals(result3, ABSOLUTE_ERROR));
            }

            if (failed) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static ECEFPositionAndVelocity computeSatellitePositionAndVelocity(
            final double time, final GNSSConfig config, final int j) {
        final var inclination = Math.toRadians(config.getSatellitesInclinationDegrees());

        final var omegaIs = Math.sqrt(Constants.EARTH_GRAVITATIONAL_CONSTANT
                / Math.pow(config.getOrbitalRadiusOfSatellites(), 3.0));

        final var constTime = time + config.getConstellationTimingOffset();

        final var uoso = 2.0 * Math.PI * j / config.getNumberOfSatellites() + omegaIs * constTime;

        final var roso = new double[]{ config.getOrbitalRadiusOfSatellites() * Math.cos(uoso),
                config.getOrbitalRadiusOfSatellites() * Math.sin(uoso) };

        final var omega = Math.PI * ((j + 1) % 6) / 3
                + Math.toRadians(config.getConstellationLongitudeOffsetDegrees())
                - Constants.EARTH_ROTATION_RATE * constTime;

        final var position = new ECEFPosition(
                roso[0] * Math.cos(omega) - roso[1] * Math.cos(inclination) * Math.sin(omega),
                roso[0] * Math.sin(omega) + roso[1] * Math.cos(inclination) * Math.cos(omega),
                roso[1] * Math.sin(inclination));

        final var voso = new double[] { - config.getOrbitalRadiusOfSatellites() * omegaIs * Math.sin(uoso),
                config.getOrbitalRadiusOfSatellites() * omegaIs * Math.cos(uoso), 0.0 };

        final var velocity = new ECEFVelocity(
                voso[0] * Math.cos(omega) - voso[1] * Math.cos(inclination) * Math.sin(omega)
                        + Constants.EARTH_ROTATION_RATE * position.getY(),
                voso[0] * Math.sin(omega) + voso[1] * Math.cos(inclination) * Math.cos(omega)
                        - Constants.EARTH_ROTATION_RATE * position.getX(),
                voso[1] * Math.sin(inclination));

        return new ECEFPositionAndVelocity(position, velocity);
    }
}
