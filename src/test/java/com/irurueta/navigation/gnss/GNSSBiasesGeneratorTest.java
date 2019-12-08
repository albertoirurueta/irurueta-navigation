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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class GNSSBiasesGeneratorTest {

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

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testGenerateBias() throws WrongSizeException {
        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final ECEFPosition satellitePosition = new ECEFPosition(
                randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
        final ECEFPosition userPosition = new ECEFPosition(
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));
        final GNSSConfig config = generateConfig();

        final double bias1 = GNSSBiasesGenerator.generateBias(satellitePosition, userPosition, config, random);
        final double bias2 = generateBias(satellitePosition, userPosition, config, random);

        assertEquals(bias1, bias2, ABSOLUTE_ERROR);
    }

    @Test
    public void testGenerateBiases() throws WrongSizeException {
        final Random random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        final GNSSConfig config = generateConfig();
        final int numSatellites = config.getNumberOfSatellites();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final ECEFPosition userPosition = new ECEFPosition(
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE),
                randomizer.nextDouble(MIN_USER_POSITION_VALUE, MAX_USER_POSITION_VALUE));

        final List<ECEFPosition> satellitePositions = new ArrayList<>();
        final List<Double> biases1 = new ArrayList<>();
        for (int i = 0; i < numSatellites; i++) {
            final ECEFPosition satellitePosition = new ECEFPosition(
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE),
                    randomizer.nextDouble(MIN_SAT_POSITION_VALUE, MAX_SAT_POSITION_VALUE));
            satellitePositions.add(satellitePosition);

            biases1.add(generateBias(satellitePosition, userPosition, config, random));
        }

        final List<Double> biases2 = new ArrayList<>();
        GNSSBiasesGenerator.generateBiases(
                satellitePositions, userPosition, config, random, biases2);
        final List<Double> biases3 = GNSSBiasesGenerator.generateBiases(
                satellitePositions, userPosition, config, random);

        assertEquals(biases2.size(), numSatellites);
        assertEquals(biases3.size(), numSatellites);
        for (int i = 0; i < numSatellites; i++) {
            assertEquals(biases1.get(i), biases2.get(i), ABSOLUTE_ERROR);
            assertEquals(biases1.get(i), biases3.get(i), ABSOLUTE_ERROR);
            assertEquals(biases2.get(i), biases3.get(i), 0.0);
        }
    }

    private static double generateBias(final ECEFPosition satellitePosition,
                                       final ECEFPosition userPosition,
                                       final GNSSConfig config, final Random random)
            throws WrongSizeException {
        final NEDPosition userNedPosition = new NEDPosition();
        final NEDVelocity userNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(userPosition, new ECEFVelocity(),
                userNedPosition, userNedVelocity);
        final double userLatitude = userNedPosition.getLatitude();
        final double userLongitude = userNedPosition.getLongitude();
        final Matrix cen = CoordinateTransformation.ecefToNedMatrix(userLatitude, userLongitude);

        final Matrix satREsE = Matrix.newFromArray(satellitePosition.getPosition().asArray());
        final Matrix rEaE = Matrix.newFromArray(userPosition.getPosition().asArray());
        final Matrix deltaR = satREsE.subtractAndReturnNew(rEaE);
        final double norm = Utils.normF(deltaR);
        final Matrix uAsE = deltaR.multiplyByScalarAndReturnNew(1.0 / norm);

        double elevation = -Math.asin(cen.getSubmatrix(
                2, 0, 2, 2)
                .multiplyAndReturnNew(uAsE).getElementAtIndex(0));

        elevation = Math.max(elevation, Math.toRadians(config.getMaskAngleDegrees()));

        final double ionoSD = config.getZenithIonosphereErrorSD()
                / Math.sqrt(1.0 - 0.899 * Math.pow(Math.cos(elevation), 2.0));
        final double tropSD = config.getZenithTroposphereErrorSD()
                / Math.sqrt(1.0 - 0.998 * Math.pow(Math.cos(elevation), 2.0));

        return config.getSISErrorSD() * random.nextGaussian()
                + ionoSD * random.nextGaussian()
                + tropSD * random.nextGaussian();
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
}
