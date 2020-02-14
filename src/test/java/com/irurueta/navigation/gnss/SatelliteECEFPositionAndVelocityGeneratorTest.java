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

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SatelliteECEFPositionAndVelocityGeneratorTest {

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
    public void testConstants() {
        assertEquals(SatelliteECEFPositionAndVelocityGenerator.EARTH_GRAVITATIONAL_CONSTANT,
                Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(SatelliteECEFPositionAndVelocityGenerator.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE,
                0.0);
    }

    @Test
    public void testGenerateSatellitesPositionAndVelocity() {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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

            final GNSSConfig config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                    rangeRateTrackingErrorSD, initialReceiverClockOffset,
                    initialReceiverClockDrift);

            final double time = randomizer.nextDouble(MIN_TIME_SECONDS, MAX_TIME_SECONDS);

            final List<ECEFPositionAndVelocity> result1 = new ArrayList<>();
            for (int j = 0; j < numberOfSatellites; j++) {
                result1.add(computeSatellitePositionAndVelocity(time, config, j));
            }

            final Collection<ECEFPositionAndVelocity> result2 =
                    SatelliteECEFPositionAndVelocityGenerator
                            .generateSatellitesPositionAndVelocity(time, config);
            final Collection<ECEFPositionAndVelocity> result3 = new ArrayList<>();
            SatelliteECEFPositionAndVelocityGenerator.generateSatellitesPositionAndVelocity(
                    time, config, result3);

            boolean failed = false;
            int j = 0;
            for (final ECEFPositionAndVelocity posVel2 : result2) {
                final ECEFPositionAndVelocity posVel1 = result1.get(j);

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
    public void testGenerateSatellitePositionAndVelocity() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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

            final GNSSConfig config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                    rangeRateTrackingErrorSD, initialReceiverClockOffset,
                    initialReceiverClockDrift);

            final double time = randomizer.nextDouble(MIN_TIME_SECONDS, MAX_TIME_SECONDS);

            boolean failed = false;
            for (int j = 0; j < numberOfSatellites; j++) {
                final ECEFPositionAndVelocity result1 = new ECEFPositionAndVelocity();
                SatelliteECEFPositionAndVelocityGenerator.generateSatellitePositionAndVelocity(time, config, j, result1);
                final ECEFPositionAndVelocity result2 = SatelliteECEFPositionAndVelocityGenerator
                        .generateSatellitePositionAndVelocity(time, config, j);

                final ECEFPositionAndVelocity result3 = computeSatellitePositionAndVelocity(time, config, j);

                if (!result1.equals(result3, LARGE_ABSOLUTE_ERROR)) {
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

    private ECEFPositionAndVelocity computeSatellitePositionAndVelocity(final double time,
                                                                                 final GNSSConfig config, final int j) {
        final double inclination = Math.toRadians(config.getSatellitesInclinationDegrees());

        final double omegaIs = Math.sqrt(Constants.EARTH_GRAVITATIONAL_CONSTANT /
                Math.pow(config.getOrbitalRadiusOfSatellites(), 3.0));

        final double constTime = time + config.getConstellationTimingOffset();

        final double uoso = 2.0 * Math.PI * j / config.getNumberOfSatellites() + omegaIs * constTime;

        final double[] roso = new double[]{ config.getOrbitalRadiusOfSatellites() * Math.cos(uoso),
                config.getOrbitalRadiusOfSatellites() * Math.sin(uoso) };

        final double omega = Math.PI * ((j + 1) % 6) / 3
                + Math.toRadians(config.getConstellationLongitudeOffsetDegrees())
                - Constants.EARTH_ROTATION_RATE * constTime;

        final ECEFPosition position = new ECEFPosition(
                roso[0] * Math.cos(omega) - roso[1] * Math.cos(inclination) * Math.sin(omega),
                roso[0] * Math.sin(omega) + roso[1] * Math.cos(inclination) * Math.cos(omega),
                roso[1] * Math.sin(inclination));

        final double[] voso = new double[] { - config.getOrbitalRadiusOfSatellites() * omegaIs * Math.sin(uoso),
                config.getOrbitalRadiusOfSatellites() * omegaIs * Math.cos(uoso), 0.0 };

        final ECEFVelocity velocity = new ECEFVelocity(
                voso[0] * Math.cos(omega) - voso[1] * Math.cos(inclination) * Math.sin(omega)
                        + Constants.EARTH_ROTATION_RATE * position.getY(),
                voso[0] * Math.sin(omega) + voso[1] * Math.cos(inclination) * Math.cos(omega)
                        - Constants.EARTH_ROTATION_RATE * position.getX(),
                voso[1] * Math.sin(inclination));

        return new ECEFPositionAndVelocity(position, velocity);
    }
}
