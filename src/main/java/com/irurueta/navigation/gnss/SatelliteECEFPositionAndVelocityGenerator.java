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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Computes satellites positions and velocities.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Satellite_positions_and_velocities.m
 */
public class SatelliteECEFPositionAndVelocityGenerator {

    /**
     * WGS84 Earth gravitational constant expressed in m^3 * s^-2
     */
    public static final double EARTH_GRAVITATIONAL_CONSTANT = Constants.EARTH_GRAVITATIONAL_CONSTANT;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Generates positions and velocities of satellites based on provided configuration.
     *
     * @param time   current time expressed in seconds (s).
     * @param config GNSS configuration.
     * @return collection containing position and velocities of satellites.
     */
    public static Collection<ECEFPositionAndVelocity> generateSatellitesPositionAndVelocity(
            final double time, final GNSSConfig config) {
        final List<ECEFPositionAndVelocity> result = new ArrayList<>();
        generateSatellitesPositionAndVelocity(time, config, result);
        return result;
    }

    /**
     * Generates positions and velocities of satellites based on provided configuration.
     *
     * @param time   current time expressed in seconds (s).
     * @param config GNSS configuration.
     * @param result instance where computed positions and velocities of satellites will be stored.
     */
    public static void generateSatellitesPositionAndVelocity(final double time, final GNSSConfig config,
                                                             final Collection<ECEFPositionAndVelocity> result) {
        result.clear();

        final int numSatellites = config.getNumberOfSatellites();
        for (int j = 0; j < numSatellites; j++) {
            final ECEFPositionAndVelocity satellitePositionAndVelocity = new ECEFPositionAndVelocity();
            generateSatellitePositionAndVelocity(time, config, j, satellitePositionAndVelocity);
            result.add(satellitePositionAndVelocity);
        }
    }

    /**
     * Generates position and velocity of a single satellite based on provided configuration.
     *
     * @param time   current time expressed in seconds (s).
     * @param config GNSS configuration.
     * @param j      number of satellite whose position and velocity must be computed.
     * @return computed satellite position and velocity.
     */
    public static ECEFPositionAndVelocity generateSatellitePositionAndVelocity(final double time,
                                                                               final GNSSConfig config,
                                                                               final int j) {
        final ECEFPositionAndVelocity result = new ECEFPositionAndVelocity();
        generateSatellitePositionAndVelocity(time, config, j, result);
        return result;
    }

    /**
     * Generates position and velocity of a single satellite based on provided configuration.
     *
     * @param time   current time expressed in seconds (s).
     * @param config GNSS configuration.
     * @param j      number of satellite whose position and velocity must be computed.
     * @param result instance where computed satellite position and velocity will be stored.
     */
    public static void generateSatellitePositionAndVelocity(final double time, final GNSSConfig config,
                                                            final int j, final ECEFPositionAndVelocity result) {

        // Convert inclination angle to radians.
        final double inclinationRadians = Math.toRadians(config.getSatellitesInclinationDegrees());

        // Determine orbital angular rate using (8.8)
        final double orbitalRadius = config.getOrbitalRadiusOfSatellites();
        final double orbitalRadius3 = orbitalRadius * orbitalRadius * orbitalRadius;
        final double omegaIs = Math.sqrt(EARTH_GRAVITATIONAL_CONSTANT / orbitalRadius3);

        // determine constellation time
        final double constTime = time + config.getConstellationTimingOffset();

        // (Corrected) argument of latitude
        final double uOsO = 2.0 * Math.PI * j / config.getNumberOfSatellites()
                + omegaIs * constTime;

        // Satellite position in the orbital frame from (8.14)
        final double cosUoso = Math.cos(uOsO);
        final double sinUoso = Math.sin(uOsO);
        final double rOsO1 = orbitalRadius * cosUoso;
        final double rOsO2 = orbitalRadius * sinUoso;

        // longitude of the ascending node from (8.16)
        final double constDeltaLambdaRadians = Math.toRadians(config.getConstellationLongitudeOffsetDegrees());
        final double omega = (Math.PI * ((j + 1) % 6) / 3.0 + constDeltaLambdaRadians)
                - EARTH_ROTATION_RATE * constTime;

        // ECEF satellite position from (8.19)
        final double cosOmega = Math.cos(omega);
        final double sinOmega = Math.sin(omega);
        final double cosInclination = Math.cos(inclinationRadians);
        final double sinInclination = Math.sin(inclinationRadians);

        final double satelliteX = rOsO1 * cosOmega
                - rOsO2 * cosInclination * sinOmega;
        final double satelliteY = rOsO1 * sinOmega
                + rOsO2 * cosInclination * cosOmega;
        final double satelliteZ = rOsO2 * sinInclination;

        // Satellite velocity in the orbital frame from (8.25), noting that with a circular orbit rOsO is
        // constant and the time derivative of uOsO is omegaIs.
        final double tmp = orbitalRadius * omegaIs;
        final double vOsO1 = -tmp * sinUoso;
        final double vOsO2 = tmp * cosUoso;

        // ECEF satellite velocity from (8.26)
        final double satelliteVx = vOsO1 * cosOmega - vOsO2 * cosInclination * sinOmega
                + EARTH_ROTATION_RATE * satelliteY;
        final double satelliteVy = vOsO1 * sinOmega + vOsO2 * cosInclination * cosOmega
                - EARTH_ROTATION_RATE * satelliteX;
        final double satelliteVz = vOsO2 * sinInclination;

        result.setPositionCoordinates(satelliteX, satelliteY, satelliteZ);
        result.setVelocityCoordinates(satelliteVx, satelliteVy, satelliteVz);
    }
}
