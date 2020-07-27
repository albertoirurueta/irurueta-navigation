/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor.position;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.statistics.MultivariateNormalDist;

import java.util.List;

/**
 * Utility class that converts located radio sources and fingerprints into positions,
 * distances and distance standard deviations that can be used to solve the lateration
 * problem.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class PositionEstimatorHelper {

    /**
     * Constructor to prevent instantiation.
     */
    private PositionEstimatorHelper() {
    }

    /**
     * Builds positions and distances from provided located radio sources and
     * fingerprint readings.
     * Notice that positions and distances lists might not have the same size
     * as provided sources list or fingerprint readings list if not all radio sources
     * between sources and fingerprint readings match.
     * If no sources, fingerprint readings, positions and distances are provided, this
     * method makes no action.
     *
     * @param sources     located radio sources to obtain positions and other
     *                    parameters.
     * @param fingerprint fingerprint containing ranged RSSI readings.
     * @param positions   list where extracted positions will be stored.
     * @param distances   list where extracted distances will be stored.
     * @param <P>         a {@link Point} type.
     */
    public static <P extends Point<?>> void buildPositionsAndDistances(
            final List<? extends RadioSourceLocated<P>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final List<P> positions, final List<Double> distances) {

        if (sources == null || fingerprint == null ||
                fingerprint.getReadings() == null ||
                positions == null || distances == null) {
            return;
        }

        positions.clear();
        distances.clear();

        final List<? extends Reading<? extends RadioSource>> readings =
                fingerprint.getReadings();
        for (final Reading<? extends RadioSource> reading : readings) {
            //noinspection all
            final int index = sources.indexOf(reading.getSource());
            if (index >= 0) {
                final RadioSourceLocated<P> locatedSource = sources.get(index);
                final P position = locatedSource.getPosition();

                //compute distance
                Double distance1 = null;
                Double distance2 = null;
                switch (reading.getType()) {
                    case RANGING_READING:
                        distance1 = computeDistanceRanging(
                                (RangingReading<? extends RadioSource>) reading);
                        break;
                    case RSSI_READING:
                        distance1 = computeDistanceRssi(locatedSource,
                                (RssiReading<? extends RadioSource>) reading);
                        break;
                    case RANGING_AND_RSSI_READING:
                        //in this case two positions and distance might be added to
                        //the lateration solver
                        distance1 = computeDistanceRanging(
                                (RangingAndRssiReading<? extends RadioSource>) reading);
                        distance2 = computeDistanceRssi(locatedSource,
                                (RangingAndRssiReading<? extends RadioSource>) reading);
                        break;
                    default:
                        break;
                }

                if (position != null) {
                    if (distance1 != null) {
                        positions.add(position);
                        distances.add(distance1);
                    }
                    if (distance2 != null) {
                        positions.add(position);
                        distances.add(distance2);
                    }
                }
            }
        }
    }

    /**
     * Builds positions, distances and standard deviations from provided located radio
     * sources and fingerprint readings.
     * Notice that positions, distances and standard deviations lists might not have
     * the same size as provided sources list or fingerprint readings list if not all
     * radio sources between sources and fingerprint readings match.
     * If no sources, fingerprint readings, positions, distances and standard deviations
     * are provided, this method makes no action.
     *
     * @param sources                           located radio sources to obtain positions
     *                                          and other parameters.
     * @param fingerprint                       fingerprint containing ranged or RSSI
     *                                          readings.
     * @param useRadioSourcePositionCovariance  true to take into account radio source
     *                                          position covariance, false otherwise.
     * @param fallbackDistanceStandardDeviation distance standard deviation to be
     *                                          assumed when it cannot be determined.
     * @param positions                         list where extracted positions will be
     *                                          stored.
     * @param distances                         list where extracted distances will be
     *                                          stored.
     * @param distanceStandardDeviations        list where extracted standard deviations
     *                                          of distances will be stored.
     * @param <P>                               a {@link Point} type.
     * @throws IllegalArgumentException if provided distance standard deviation fallback
     *                                  is negative.
     */
    public static <P extends Point<?>> void buildPositionsDistancesAndDistanceStandardDeviations(
            final List<? extends RadioSourceLocated<P>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final boolean useRadioSourcePositionCovariance,
            final double fallbackDistanceStandardDeviation,
            final List<P> positions, final List<Double> distances,
            final List<Double> distanceStandardDeviations) {
        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(sources,
                fingerprint, null, null, useRadioSourcePositionCovariance,
                fallbackDistanceStandardDeviation, positions, distances,
                distanceStandardDeviations, null);
    }

    /**
     * Builds positions, distances and standard deviations from provided located radio
     * sources and fingerprint readings.
     * Notice that positions, distance and standard deviations lists might not have the
     * same size as provided sources list or fingerprint readings list if not all radio
     * sources between sources and fingerprint readings match.
     * If no sources, fingerprint readings, positions, distances and standard deviations
     * are provided, this method makes no action.
     *
     * @param sources                           located radio sources to obtain
     *                                          positions and other parameters.
     * @param fingerprint                       fingerprint containing ranged or RSSI
     *                                          readings.
     * @param sourceQualityScores               quality scores corresponding to each
     *                                          provided located radio source. The larger
     *                                          the score value the better the quality of
     *                                          the sample. If null, no quality scores
     *                                          will be stored.
     * @param fingerprintReadingsQualityScores  quality scores corresponding to each
     *                                          reading within provided fingerprint.
     * @param useRadioSourcePositionCovariance  true to take into account radio source
     *                                          position covariance, false otherwise.
     * @param fallbackDistanceStandardDeviation distance standard deviation to be
     *                                          assumed when it cannot be determined.
     * @param positions                         list where extracted positions will be stored.
     * @param distances                         list where extracted distances will be stored.
     * @param distanceStandardDeviations        list where extracted standard deviations of
     *                                          distances will be stored.
     * @param distanceQualityScores             list where extracted quality scores will
     *                                          be stored. If null, quality scores will
     *                                          be ignored.
     * @param <P>                               a {@link Point} type.
     * @throws IllegalArgumentException if provided distance standard deviation
     *                                  fallback is negative.
     */
    public static <P extends Point<?>> void buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            final List<? extends RadioSourceLocated<P>> sources,
            final Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            final double[] sourceQualityScores, final double[] fingerprintReadingsQualityScores,
            final boolean useRadioSourcePositionCovariance,
            final double fallbackDistanceStandardDeviation,
            final List<P> positions, final List<Double> distances,
            final List<Double> distanceStandardDeviations,
            final List<Double> distanceQualityScores) {

        if (fallbackDistanceStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        if (sources == null || fingerprint == null ||
                fingerprint.getReadings() == null ||
                positions == null || distances == null ||
                distanceStandardDeviations == null) {
            return;
        }

        positions.clear();
        distances.clear();
        distanceStandardDeviations.clear();

        if ((sourceQualityScores != null || fingerprintReadingsQualityScores != null) &&
                distanceQualityScores != null) {
            distanceQualityScores.clear();
        }

        final Double[] result1 = new Double[2];
        final Double[] result2 = new Double[2];

        final List<? extends Reading<? extends RadioSource>> readings =
                fingerprint.getReadings();
        int readingIndex = 0;
        for (final Reading<? extends RadioSource> reading : readings) {
            // noinspection all
            final int sourceIndex = sources.indexOf(reading.getSource());
            final Double readingQualityScore = fingerprintReadingsQualityScores != null ?
                    fingerprintReadingsQualityScores[readingIndex] : null;
            Double sourceQualityScore = null;
            Double qualityScore = null;
            if (sourceIndex >= 0) {
                final RadioSourceLocated<P> locatedSource = sources.get(sourceIndex);
                final P position = locatedSource.getPosition();
                if (sourceQualityScores != null) {
                    sourceQualityScore = sourceQualityScores[sourceIndex];
                }
                readingIndex++;

                if (readingQualityScore != null || sourceQualityScore != null) {
                    qualityScore = 0.0;
                    if (readingQualityScore != null) {
                        qualityScore += readingQualityScore;
                    }
                    if (sourceQualityScore != null) {
                        qualityScore += sourceQualityScore;
                    }
                }

                Matrix positionCovariance = null;
                if (useRadioSourcePositionCovariance) {
                    positionCovariance = locatedSource.getPositionCovariance();
                }

                Double positionStandardDeviation = null;
                if (positionCovariance != null) {
                    try {
                        // compute standard deviation associated to position
                        // uncertainty
                        final SingularValueDecomposer decomposer =
                                new SingularValueDecomposer(positionCovariance);
                        decomposer.decompose();

                        // singular values contain variances on each principal axis
                        final double[] singularValues = decomposer.getSingularValues();

                        // compute average of singular values as an "average" variance
                        // of position
                        double variance = 0.0;
                        for (final double singularValue : singularValues) {
                            variance += singularValue / singularValues.length;
                        }

                        positionStandardDeviation = Math.sqrt(variance);

                    } catch (final AlgebraException e) {
                        positionStandardDeviation = null;
                    }
                }

                // compute distance and standard deviation
                result1[0] = result1[1] = result2[0] = result2[1] = null;
                switch (reading.getType()) {
                    case RANGING_READING:
                        computeDistanceAndStandardDeviationRanging(
                                (RangingReading<? extends RadioSource>) reading,
                                positionStandardDeviation, result1);
                        break;
                    case RSSI_READING:
                        computeDistanceAndStandardDeviationRssi(locatedSource,
                                (RssiReading<? extends RadioSource>) reading,
                                positionStandardDeviation, result1);
                        break;
                    case RANGING_AND_RSSI_READING:
                        computeDistanceAndStandardDeviationRanging(
                                (RangingAndRssiReading<? extends RadioSource>) reading,
                                positionStandardDeviation, result1);
                        computeDistanceAndStandardDeviationRssi(locatedSource,
                                (RangingAndRssiReading<? extends RadioSource>) reading,
                                positionStandardDeviation, result2);
                        break;
                    default:
                        break;
                }

                if (position != null) {
                    final Double distance1 = result1[0];
                    final Double distance2 = result2[0];
                    if (distance1 != null) {
                        final Double standardDeviation1 = result1[1];

                        positions.add(position);
                        distances.add(distance1);
                        distanceStandardDeviations.add(standardDeviation1 != null ?
                                standardDeviation1 : fallbackDistanceStandardDeviation);

                        if (qualityScore != null && distanceQualityScores != null) {
                            distanceQualityScores.add(qualityScore);
                        }
                    }

                    if (distance2 != null) {
                        final Double standardDeviation2 = result2[1];

                        positions.add(position);
                        distances.add(distance2);
                        distanceStandardDeviations.add(standardDeviation2 != null ?
                                standardDeviation2 : fallbackDistanceStandardDeviation);

                        if (qualityScore != null && distanceQualityScores != null) {
                            distanceQualityScores.add(qualityScore);
                        }
                    }
                }
            }
        }
    }

    /**
     * Obtains distance for a ranging reading.
     *
     * @param reading a ranging reading.
     * @return distance to reading source or null if not available.
     */
    private static Double computeDistanceRanging(
            final RangingReading<? extends RadioSource> reading) {
        return reading.getDistance();
    }

    /**
     * Obtains distance for a ranging reading.
     *
     * @param reading a ranging reading.
     * @return distance to reading source or null if not available.
     */
    private static Double computeDistanceRanging(
            final RangingAndRssiReading<? extends RadioSource> reading) {
        return reading.getDistance();
    }

    /**
     * Obtains distance for an RSSI reading.
     *
     * @param locatedSource a located source, that must also have power information.
     * @param reading       an RSSI reading.
     * @param <P>           a {@link Point} type.
     * @return estimated distance or null if not available.
     */
    private static <P extends Point<?>> Double computeDistanceRssi(
            final RadioSourceLocated<P> locatedSource,
            final RssiReading<? extends RadioSource> reading) {
        return computeDistanceRssi(locatedSource, reading.getRssi());
    }

    /**
     * Obtains distance for a ranging and RSSI reading.
     *
     * @param locatedSource a located source, that must also have power information.
     * @param reading       a ranging and RSSI reading.
     * @param <P>           a {@link Point} type.
     * @return estimated distance or null if not available.
     */
    private static <P extends Point<?>> Double computeDistanceRssi(
            final RadioSourceLocated<P> locatedSource,
            final RangingAndRssiReading<? extends RadioSource> reading) {
        return computeDistanceRssi(locatedSource, reading.getRssi());
    }

    /**
     * Obtains distance.
     *
     * @param locatedSource a located source, that must also have power information.
     * @param rxPower       ;
     * @param <P>           a {@link Point} type.
     * @return estimated distance or null if not available.
     */
    private static <P extends Point<?>> Double computeDistanceRssi(
            final RadioSourceLocated<P> locatedSource, final double rxPower) {
        if (!(locatedSource instanceof RadioSourceWithPower)) {
            return null;
        }

        final RadioSourceWithPower poweredSource = (RadioSourceWithPower) locatedSource;

        //source related parameters:

        //transmitted power in dBm's
        final double txPower = poweredSource.getTransmittedPower();

        //path loss exponent
        final double pathLossExponent = poweredSource.getPathLossExponent();

        final double frequency = poweredSource.getFrequency();
        final double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        final double kdB = 10.0 * Math.log10(k);


        //received power in dBm's follows the equation:
        //rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance

        //hence:
        //5.0 * pathLossExponent * logSqrDistance = pathLossExponent * kdB + txPower - rxPower

        final double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower) / (5.0 * pathLossExponent);

        //where logSqrDistance = Math.log10(sqrDistance)
        //and sqrDistance = distance * distance, hence
        //logSqrDistance = Math.log10(distance * distance) = 2 * Math.log10(distance)

        return Math.pow(10.0, logSqrDistance / 2.0);
    }

    /**
     * Obtains distance and its standard deviation for a ranging reading.
     *
     * @param reading                   a ranging reading.
     * @param positionStandardDeviation position standard deviation, or null if
     *                                  not available.
     * @param result                    array containing distance and its standard
     *                                  deviation, in such order.
     */
    private static void computeDistanceAndStandardDeviationRanging(
            final RangingReading<? extends RadioSource> reading,
            final Double positionStandardDeviation, final Double[] result) {
        computeDistanceAndStandardDeviationRanging(reading.getDistance(),
                reading.getDistanceStandardDeviation(),
                positionStandardDeviation, result);
    }

    /**
     * Obtains distance and its standard deviation for a ranging reading.
     *
     * @param reading                   a ranging reading.
     * @param positionStandardDeviation position standard deviation, or null if
     *                                  not available
     * @param result                    array containing distance and its standard
     *                                  deviation, in such order.
     */
    private static void computeDistanceAndStandardDeviationRanging(
            final RangingAndRssiReading<? extends RadioSource> reading,
            final Double positionStandardDeviation, final Double[] result) {
        computeDistanceAndStandardDeviationRanging(reading.getDistance(),
                reading.getDistanceStandardDeviation(),
                positionStandardDeviation, result);
    }

    /**
     * Obtains distance and its standard deviation.
     *
     * @param distance                  a distance.
     * @param distanceStandardDeviation distance standard deviation.
     * @param positionStandardDeviation position standard deviation.
     * @param result                    array containing distance and its standard
     *                                  deviation, in such order.
     */
    private static void computeDistanceAndStandardDeviationRanging(
            final double distance, final Double distanceStandardDeviation,
            final Double positionStandardDeviation, final Double[] result) {
        result[0] = distance;

        if (positionStandardDeviation != null || distanceStandardDeviation != null) {
            double variance = 0.0;
            if (positionStandardDeviation != null) {
                variance += positionStandardDeviation * positionStandardDeviation;
            }
            if (distanceStandardDeviation != null) {
                variance += distanceStandardDeviation * distanceStandardDeviation;
            }
            result[1] = Math.sqrt(variance);
        } else {
            result[1] = null;
        }
    }

    /**
     * Obtains distance and its standard deviation for an RSSI reading.
     *
     * @param locatedSource             a located source, that must also have pwoer
     *                                  information.
     * @param reading                   an RSSI reading.
     * @param positionStandardDeviation position standard deviation, or null if not
     *                                  available.
     * @param result                    array containing distance and its standard
     *                                  deviation, in such order.
     * @param <P>                       a {@link Point} type.
     */
    private static <P extends Point<?>> void computeDistanceAndStandardDeviationRssi(
            final RadioSourceLocated<P> locatedSource,
            final RssiReading<? extends RadioSource> reading,
            final Double positionStandardDeviation, final Double[] result) {
        computeDistanceAndStandardDeviationRssi(locatedSource, reading.getRssi(),
                reading.getRssiStandardDeviation(), positionStandardDeviation,
                result);
    }

    /**
     * Obtains distance and its standard deviation for a ranging and RSSI reading.
     *
     * @param locatedSource             a located source, that must also have power
     *                                  information.
     * @param reading                   a ranging and RSSI reading.
     * @param positionStandardDeviation position standard deviation, or null if not
     *                                  available.
     * @param result                    array containing distance and its standard
     *                                  deviation, in such order.
     * @param <P>                       a {@link Point} type.
     */
    private static <P extends Point<?>> void computeDistanceAndStandardDeviationRssi(
            final RadioSourceLocated<P> locatedSource,
            final RangingAndRssiReading<? extends RadioSource> reading,
            final Double positionStandardDeviation, final Double[] result) {
        computeDistanceAndStandardDeviationRssi(locatedSource, reading.getRssi(),
                reading.getRssiStandardDeviation(), positionStandardDeviation,
                result);
    }

    /**
     * Obtains distance and its standard deviation.
     *
     * @param locatedSource             a located source, that must also have power
     *                                  information.
     * @param rxPower                   received power expressed in dBm's.
     * @param rxPowerStandardDeviation  received power standard deviation.
     * @param positionStandardDeviation position standard deviation.
     * @param result                    array containing distance and its standard
     *                                  deviation, in such order.
     * @param <P>                       a {@link Point} type.
     */
    private static <P extends Point<?>> void computeDistanceAndStandardDeviationRssi(
            final RadioSourceLocated<P> locatedSource, final double rxPower,
            final Double rxPowerStandardDeviation,
            final Double positionStandardDeviation,
            final Double[] result) {

        if (!(locatedSource instanceof RadioSourceWithPower)) {
            return;
        }

        final RadioSourceWithPower poweredSource = (RadioSourceWithPower) locatedSource;

        // source related parameters
        // transmitted power in dBm's
        final double txPower = poweredSource.getTransmittedPower();
        final Double txPowerStandardDeviation =
                poweredSource.getTransmittedPowerStandardDeviation();

        // path loss exponent
        final double pathLossExponent = poweredSource.getPathLossExponent();
        final Double pathLossExponentStandardDeviation =
                poweredSource.getPathLossExponentStandardDeviation();

        //WARNING: covariance between tx power and path loss exponent is ignored

        final double frequency = poweredSource.getFrequency();

        final double txPowerVariance = txPowerStandardDeviation != null ?
                txPowerStandardDeviation * txPowerStandardDeviation : 0.0;
        final double rxPowerVariance = rxPowerStandardDeviation != null ?
                rxPowerStandardDeviation * rxPowerStandardDeviation : 0.0;
        final double pathLossVariance = pathLossExponentStandardDeviation != null ?
                pathLossExponentStandardDeviation * pathLossExponentStandardDeviation :
                0.0;

        double distanceVariance = 0.0;
        try {
            final MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPower, rxPower, pathLossExponent, frequency, txPowerVariance,
                    rxPowerVariance, pathLossVariance);
            // distance
            result[0] = dist.getMean()[0];

            // distance variance
            distanceVariance = dist.getCovariance().getElementAt(0, 0);
        } catch (final IndoorException e) {
            result[0] = computeDistanceRssi(locatedSource, rxPower);
            if (rxPowerStandardDeviation != null) {
                // take into account only received power standard deviation
                distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(
                        txPower, rxPower, pathLossExponent, frequency, rxPowerVariance);
            }
        }

        if (txPowerStandardDeviation == null && rxPowerStandardDeviation == null &&
                pathLossExponentStandardDeviation == null &&
                positionStandardDeviation == null) {
            result[1] = null;
        } else {
            if (positionStandardDeviation != null) {
                distanceVariance += positionStandardDeviation * positionStandardDeviation;
            }
            result[1] = Math.sqrt(distanceVariance);
        }
    }
}
