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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Generates the GNSS range errors due to signal in space, ionosphere and troposphere
 * errors based on the elevation angles.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multi-sensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_GNSS_biases.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/Initialize_GNSS_biases.m
 * </a>
 */
public class GNSSBiasesGenerator {

    /**
     * Ionosphere factor.
     */
    private static final double IONO_FACTOR = 0.899;

    /**
     * Troposphere factor.
     */
    private static final double TROPO_FACTOR = 0.998;

    /**
     * Constructor.
     * Prevents instantiation of utility class.
     */
    private GNSSBiasesGenerator() {
    }

    /**
     * Generates biases.
     *
     * @param satellitePositions ECEF satellite positions expressed in meters (m).
     * @param userPosition       ECEF user position expressed in meters (m).
     * @param config             GNSS configuration.
     * @param random             random number generator.
     * @return list of generated biases for each provided satellite position.
     */
    public static List<Double> generateBiases(final List<ECEFPosition> satellitePositions,
                                              final ECEFPosition userPosition,
                                              final GNSSConfig config, final Random random) {
        final List<Double> result = new ArrayList<>();
        generateBiases(satellitePositions, userPosition, config, random, result);
        return result;
    }

    /**
     * Generates biases.
     *
     * @param satellitePositions ECEF satellite positions expressed in meters (m).
     * @param userPosition       ECEF user position expressed in meters (m).
     * @param config             GNSS configuration.
     * @param random             random number generator.
     * @param result             instance where generated biases for each
     *                           provided satellite position will be stored.
     */
    public static void generateBiases(final List<ECEFPosition> satellitePositions,
                                      final ECEFPosition userPosition,
                                      final GNSSConfig config, final Random random,
                                      final List<Double> result) {
        // Calculate NED user position
        final NEDPosition userNedPosition = new NEDPosition();
        final NEDVelocity userNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                userPosition.getX(), userPosition.getY(), userPosition.getZ(),
                0.0, 0.0, 0.0, userNedPosition, userNedVelocity);
        generate(satellitePositions, userPosition,
                userNedPosition.getLatitude(), userNedPosition.getLongitude(),
                config, random, result);
    }

    /**
     * Generates biases.
     *
     * @param satellitePosition ECEF satellite positions expressed in meters (m).
     * @param userPosition      ECEF user position expressed in meters (m).
     * @param config            GNSS configuration.
     * @param random            random number generator.
     * @return generated bias for provided satellite position.
     */
    public static double generateBias(final ECEFPosition satellitePosition,
                                      final ECEFPosition userPosition, final GNSSConfig config,
                                      final Random random) {
        // Calculate NED user position
        final NEDPosition userNedPosition = new NEDPosition();
        final NEDVelocity userNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                userPosition.getX(), userPosition.getY(), userPosition.getZ(),
                0.0, 0.0, 0.0, userNedPosition, userNedVelocity);
        final double userLatitude = userNedPosition.getLatitude();
        final double userLongitude = userNedPosition.getLongitude();

        // Calculate ECEF to NED coordinate transformation matrix
        final Matrix cen = CoordinateTransformation.ecefToNedMatrix(userLatitude, userLongitude);
        return generate(satellitePosition, userPosition, config, cen, random);
    }

    /**
     * Generates biases.
     *
     * @param satellitePositions ECEF satellite positions expressed in meters (m).
     * @param userPosition       ECEF user position expressed in meters (m).
     * @param userLatitude       user latitude expressed in radians (rad).
     * @param userLongitude      user longitude expressed in radians (rad).
     * @param config             GNSS configuration.
     * @param random             random number generator.
     * @param result             instance where generated biases for each
     *                           provided satellite position will be stored.
     */
    private static void generate(final List<ECEFPosition> satellitePositions,
                                 final ECEFPosition userPosition,
                                 final double userLatitude,
                                 final double userLongitude,
                                 final GNSSConfig config, final Random random,
                                 final List<Double> result) {
        result.clear();

        // Calculate ECEF to NED coordinate transformation matrix
        final Matrix cen = CoordinateTransformation.ecefToNedMatrix(userLatitude, userLongitude);

        // Loop satellites
        for (final ECEFPosition satellitePosition : satellitePositions) {
            result.add(generate(satellitePosition, userPosition, config, cen, random));
        }
    }

    /**
     * Generates bias.
     *
     * @param satellitePosition ECEF satellite position expressed in meters (m).
     * @param userPosition      ECEF user position expressed in meters (m).
     * @param config            GNSS configuration.
     * @param cen               ECEF to NED coordinate transformation matrix.
     * @param random            random number generator.
     * @return generated bias provided satellite position.
     */
    @SuppressWarnings("DuplicatedCode")
    private static double generate(final ECEFPosition satellitePosition, final ECEFPosition userPosition,
                                   final GNSSConfig config, final Matrix cen, final Random random) {

        // Determine ECEF line-of-sight vector using (8.41)
        final double deltaRx = satellitePosition.getX() - userPosition.getX();
        final double deltaRy = satellitePosition.getY() - userPosition.getY();
        final double deltaRz = satellitePosition.getZ() - userPosition.getZ();

        final double deltaRNorm = Math.sqrt(deltaRx * deltaRx + deltaRy * deltaRy + deltaRz * deltaRz);

        final double uaseX = deltaRx / deltaRNorm;
        final double uaseY = deltaRy / deltaRNorm;
        final double uaseZ = deltaRz / deltaRNorm;

        // Convert line of sight vector to NED using (8.39) and determine
        // elevation using (8.57)
        final double cen1 = cen.getElementAt(2, 0);
        final double cen2 = cen.getElementAt(2, 1);
        final double cen3 = cen.getElementAt(2, 2);

        double elevation = -Math.asin(cen1 * uaseX + cen2 * uaseY + cen3 * uaseZ);

        // Limit the minimum elevation angle to the masking angle
        elevation = Math.max(elevation, Math.toRadians(config.getMaskAngleDegrees()));

        // Calculate ionosphere and troposphere error SDs using (9.79) and (9.80)
        final double cosElevation = Math.cos(elevation);
        final double cosElevation2 = cosElevation * cosElevation;
        final double ionoSD = config.getZenithIonosphereErrorSD() / Math.sqrt(1.0
                - IONO_FACTOR * cosElevation2);
        final double tropSD = config.getZenithTroposphereErrorSD() / Math.sqrt(1.0
                - TROPO_FACTOR * cosElevation2);

        // Determine range bias
        return config.getSISErrorSD() * random.nextGaussian() + ionoSD * random.nextGaussian()
                + tropSD * random.nextGaussian();
    }
}
