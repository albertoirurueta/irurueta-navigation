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
package com.irurueta.navigation.indoor.oldposition;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.statistics.MultivariateNormalDist;

import java.util.List;

/**
 * Utility class that converts located radio sources and
 * fingerprints into positions, distances and distance standard deviations that can be
 * used to solve the trilateration problem.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public class PositionEstimatorHelper {

    /**
     * Constructor to prevent instantiation.
     */
    private PositionEstimatorHelper() { }

    /**
     * Builds positions and distances from provided located radio sources and
     * fingerprint readings.
     * Notice that positions and distances lists might not have the same size
     * as provided sources list or fingerprint readings list if not all radio sources
     * between sources and fingerprint readings match.
     * If no sources, fingerprint readings, positions and distances are provided, this
     * method makes no action.
     * @param sources located radio sources to obtain positions and other parameters.
     * @param fingerprint fingerprint containing ranged RSSI readings.
     * @param positions list where extracted positions will be stored.
     * @param distances list where extracted distances will be stored.
     * @param <P> a {@link Point} type.
     */
    public static <P extends Point> void buildPositionsAndDistances(
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            List<P> positions, List<Double> distances) {

        if (sources == null || fingerprint == null ||
                fingerprint.getReadings() == null ||
                positions == null || distances == null) {
            return;
        }

        positions.clear();
        distances.clear();

        List<? extends Reading<? extends RadioSource>> readings =
                fingerprint.getReadings();
        for (Reading<? extends RadioSource> reading : readings) {
            //noinspection all
            int index = sources.indexOf(reading.getSource());
            if (index >= 0) {
                RadioSourceLocated<P> locatedSource = sources.get(index);
                P position = locatedSource.getPosition();

                //compute distance
                Double distance1 = null;
                Double distance2 = null;
                switch (reading.getType()) {
                    case RANGING_READING:
                        distance1 = computeDistanceRanging(
                                (RangingReading<? extends RadioSource>)reading);
                        break;
                    case RSSI_READING:
                        distance1 =  computeDistanceRssi(locatedSource,
                                (RssiReading<? extends RadioSource>)reading);
                        break;
                    case RANGING_AND_RSSI_READING:
                        //in this case two positions and distance might be added to
                        //the trilateration solver
                        distance1 = computeDistanceRanging(
                                (RangingAndRssiReading<? extends RadioSource>)reading);
                        distance2 =  computeDistanceRssi(locatedSource,
                                (RangingAndRssiReading<? extends RadioSource>)reading);
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
     * Builds positions, distances and standard deviations from provided locatd radio
     * sources and fingerprint readings.
     * Notice that positions, distances and standard deviations lists might not have the
     * same size as provided sources list or fingerprint readings list if not all radio
     * sources between sources and fingerprint readings match.
     * If no sources, fingerprint readings, positions, distances and standard deviations
     * are provided, this method makes no action.
     * @param sources located radio sources to obtain positions and other parameters.
     * @param fingerprint fingerprint containing ranged RSSI readings.
     * @param useRadioSourcePositionCovariance true to take into account radio source
     *                                         position covariance, false otherwise.
     * @param fallbackDistanceStandardDeviation distance standard deviation to be assumed
     *                                          when it cannot be determined.
     * @param positions list where extracted positions will be stored.
     * @param distances list where extracted distances will be stored.
     * @param distanceStandardDeviations list where extracted standard deviations of
     *                                   distances will be stored.
     * @throws IllegalArgumentException if provided distance standard deviation
     * fallback is negative.
     * @param <P> a {@link Point} type.
     */
    public static <P extends Point> void buildPositionsDistancesAndDistanceStandardDeviations(
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            boolean useRadioSourcePositionCovariance,
            double fallbackDistanceStandardDeviation,
            List<P> positions, List<Double> distances,
            List<Double> distanceStandardDeviations) {
        buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(sources, fingerprint,
                null, useRadioSourcePositionCovariance,
                fallbackDistanceStandardDeviation, positions, distances,
                distanceStandardDeviations, null);
    }

    /**
     * Builds positions, distances and standard deviations from provided locatd radio
     * sources and fingerprint readings.
     * Notice that positions, distances and standard deviations lists might not have the
     * same size as provided sources list or fingerprint readings list if not all radio
     * sources between sources and fingerprint readings match.
     * If no sources, fingerprint readings, positions, distances and standard deviations
     * are provided, this method makes no action.
     * @param sources located radio sources to obtain positions and other parameters.
     * @param fingerprint fingerprint containing ranged RSSI readings.
     * @param qualityScores quality scores corresponding to each provided located
     *                      radio source. The larger the score value the better the
     *                      quality of the sample. If null, no distance quality
     *                      scores will be stored.
     * @param useRadioSourcePositionCovariance true to take into account radio source
     *                                         position covariance, false otherwise.
     * @param fallbackDistanceStandardDeviation distance standard deviation to be assumed
     *                                          when it cannot be determined.
     * @param positions list where extracted positions will be stored.
     * @param distances list where extracted distances will be stored.
     * @param distanceStandardDeviations list where extracted standard deviations of
     *                                   distances will be stored.
     * @param distanceQualityScores list where extracted quality scores will be stored.
     *                              If null, quality scores will be ignored.
     * @throws IllegalArgumentException if provided distance standard deviation
     * fallback is negative.
     * @param <P> a {@link Point} type.
     */
    public static <P extends Point> void buildPositionsDistancesDistanceStandardDeviationsAndQualityScores(
            List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            double[] qualityScores,
            boolean useRadioSourcePositionCovariance,
            double fallbackDistanceStandardDeviation,
            List<P> positions, List<Double> distances,
            List<Double> distanceStandardDeviations,
            List<Double> distanceQualityScores) {

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

        if (qualityScores != null && distanceQualityScores != null) {
            distanceQualityScores.clear();
        }

        Double[] result1 = new Double[2];
        Double[] result2 = new Double[2];

        List<? extends Reading<? extends RadioSource>> readings =
                fingerprint.getReadings();
        for (Reading<? extends RadioSource> reading : readings) {
            //noinspection all
            int index = sources.indexOf(reading.getSource());
            Double qualityScore = null;
            if (index >= 0) {
                RadioSourceLocated<P> locatedSource = sources.get(index);
                P position = locatedSource.getPosition();
                if (qualityScores != null) {
                    qualityScore = qualityScores[index];
                }

                Matrix positionCovariance = null;
                if (useRadioSourcePositionCovariance) {
                    positionCovariance = locatedSource.getPositionCovariance();
                }

                Double positionStandardDeviation = null;
                if (positionCovariance != null) {
                    try {
                        //compute standard deviation associated to position uncertainty
                        SingularValueDecomposer decomposer =
                                new SingularValueDecomposer(positionCovariance);
                        decomposer.decompose();

                        //singular values contain variances on each principal axis
                        double[] singularValues = decomposer.getSingularValues();

                        //compute average of singular values as an "average" variance
                        //of position
                        double variance = 0.0;
                        for (double singularValue : singularValues) {
                            variance += singularValue / singularValues.length;
                        }

                        positionStandardDeviation = Math.sqrt(variance);

                    } catch (AlgebraException e) {
                        positionStandardDeviation = null;
                    }
                }

                //compute distance and standard deviation
                result1[0] = result1[1] = result2[0] = result2[1] = null;
                switch (reading.getType()) {
                    case RANGING_READING:
                        computeDistanceAndStandardDeviationRanging(
                                (RangingReading<? extends RadioSource>)reading,
                                positionStandardDeviation, result1);
                        break;
                    case RSSI_READING:
                        computeDistanceAndStandardDeviationRssi(locatedSource,
                                (RssiReading<? extends RadioSource>) reading,
                                positionStandardDeviation, result1);
                        break;
                    case RANGING_AND_RSSI_READING:
                        computeDistanceAndStandardDeviationRanging(
                                (RangingAndRssiReading<? extends RadioSource>)reading,
                                positionStandardDeviation, result1);
                        computeDistanceAndStandardDeviationRssi(locatedSource,
                                (RangingAndRssiReading<? extends RadioSource>) reading,
                                positionStandardDeviation, result2);
                        break;
                    default:
                        break;
                }

                if (position != null) {
                    Double distance1 = result1[0];
                    Double distance2 = result2[0];
                    if (distance1 != null) {
                        Double standardDeviation1 = result1[1];

                        positions.add(position);
                        distances.add(distance1);
                        distanceStandardDeviations.add(standardDeviation1 != null ?
                                standardDeviation1 : fallbackDistanceStandardDeviation);

                        if (qualityScore != null && distanceQualityScores != null) {
                            distanceQualityScores.add(qualityScore);
                        }
                    }
                    if (distance2 != null) {
                        Double standardDeviation2 = result2[1];

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
     * @param reading a ranging reading.
     * @return distance to reading source or null if not available.
     */
    private static Double computeDistanceRanging(
            RangingReading<? extends RadioSource> reading) {
        return reading.getDistance();
    }

    /**
     * Obtains distance for a ranging reading.
     * @param reading a ranging reading.
     * @return distance to reading source or null if not available.
     */
    private static Double computeDistanceRanging(
            RangingAndRssiReading<? extends RadioSource> reading) {
        return reading.getDistance();
    }

    /**
     * Obtains distance for an RSSI reading.
     * @param locatedSource a located source, that must also have power information.
     * @param reading an RSSI reading.
     * @return estimated distance or null if not available.
     * @param <P> a {@link Point} type.
     */
    private static <P extends Point> Double computeDistanceRssi(
            RadioSourceLocated<P> locatedSource,
            RssiReading<? extends RadioSource> reading) {
        return computeDistanceRssi(locatedSource, reading.getRssi());
    }

    /**
     * Obtains distance for a ranging and RSSI reading.
     * @param locatedSource a located source, that must also have power information.
     * @param reading a ranging and RSSI reading.
     * @return estimated distance or null if not available.
     * @param <P> a {@link Point} type.
     */
    private static <P extends Point> Double computeDistanceRssi(
            RadioSourceLocated<P> locatedSource,
            RangingAndRssiReading<? extends RadioSource> reading) {
        return computeDistanceRssi(locatedSource, reading.getRssi());
    }

    /**
     * Obtains distance.
     * @param locatedSource a located source, that must also have power information.
     * @param rxPower ;
     * @return estimated distance or null if not available.
     * @param <P> a {@link Point} type.
     */
    private static <P extends Point> Double computeDistanceRssi(
            RadioSourceLocated<P> locatedSource, double rxPower) {
        if(!(locatedSource instanceof RadioSourceWithPower)) {
            return null;
        }

        RadioSourceWithPower poweredSource = (RadioSourceWithPower)locatedSource;

        //source related parameters:

        //transmitted power in dBm's
        double txPower = poweredSource.getTransmittedPower();

        //path loss exponent
        double pathLossExponent = poweredSource.getPathLossExponent();

        double frequency = poweredSource.getFrequency();
        double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        double kdB = 10.0 * Math.log10(k);


        //received power in dBm's follows the equation:
        //rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance

        //hence:
        //5.0 * pathLossExponent * logSqrDistance = pathLossExponent * kdB + txPower - rxPower

        double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower) / (5.0 * pathLossExponent);

        //where logSqrDistance = Math.log10(sqrDistance)
        //and sqrDistance = distance * distance, hence
        //logSqrDistance = Math.log10(distance * distance) = 2 * Math.log10(distance)

        return Math.pow(10.0,logSqrDistance / 2.0);
    }


    /**
     * Obtains distance and its standard deviation for a ranging reading.
     * @param reading a ranging reading.
     * @param positionStandardDeviation position standard deviation, or null
     *                                  if not available.
     * @param result array containing distance and its standard deviation, in such
     *               order.
     */
    private static void computeDistanceAndStandardDeviationRanging(
            RangingReading<? extends RadioSource> reading,
            Double positionStandardDeviation, Double[] result) {
        computeDistanceAndStandardDeviationRanging(reading.getDistance(),
                reading.getDistanceStandardDeviation(),
                positionStandardDeviation, result);
    }

    /**
     * Obtains distance and its standard deviation for a ranging reading.
     * @param reading a ranging reading.
     * @param positionStandardDeviation position standard deviation, or null
     *                                  if not available.
     * @param result array containing distance and its standard deviation, in such
     *               order.
     */
    private static void computeDistanceAndStandardDeviationRanging(
            RangingAndRssiReading<? extends RadioSource> reading,
            Double positionStandardDeviation, Double[] result) {
        computeDistanceAndStandardDeviationRanging(reading.getDistance(),
                reading.getDistanceStandardDeviation(),
                positionStandardDeviation, result);
    }

    /**
     * Obtains distance and its standard deviation.
     * @param distance a distance.
     * @param distanceStandardDeviation distance standard deviation.
     * @param positionStandardDeviation position standard deviation
     * @param result array containing distance and its standard deviation, in such
     *               order.
     */
    private static void computeDistanceAndStandardDeviationRanging(double distance,
            Double distanceStandardDeviation, Double positionStandardDeviation,
            Double[] result) {
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
     * @param locatedSource a located source, that must also have power information.
     * @param reading an RSSI reading.
     * @param positionStandardDeviation position standard deviation, or null
     *                                  if not available.
     * @param result array containing distance and its standard deviation, in such
     *               order.
     * @param <P> a {@link Point} type.
     */
    private static <P extends Point> void computeDistanceAndStandardDeviationRssi(
            RadioSourceLocated<P> locatedSource,
            RssiReading<? extends RadioSource> reading,
            Double positionStandardDeviation, Double[] result) {
        computeDistanceAndStandardDeviationRssi(locatedSource, reading.getRssi(),
                reading.getRssiStandardDeviation(), positionStandardDeviation, result);
    }

    /**
     * Obtains distance and its standard deviation for a ranging and RSSI reading.
     * @param locatedSource a located source, that must also have power information.
     * @param reading a ranging and RSSI reading.
     * @param positionStandardDeviation position standard deviation, or null
     *                                  if not available.
     * @param result array containing distance and its standard deviation, in such
     *               order.
     * @param <P> a {@link Point} type.
     */
    private static <P extends Point> void computeDistanceAndStandardDeviationRssi(
            RadioSourceLocated<P> locatedSource,
            RangingAndRssiReading<? extends RadioSource> reading,
            Double positionStandardDeviation, Double[] result) {
        computeDistanceAndStandardDeviationRssi(locatedSource, reading.getRssi(),
                reading.getRssiStandardDeviation(), positionStandardDeviation, result);
    }

    /**
     * Obtains distance and its standard deviation.
     * @param locatedSource a located source, that must also have power information.
     * @param rxPower received power expressed in dBm's.
     * @param rxPowerStandardDeviation received power standard deviation.
     * @param positionStandardDeviation position standard deviation.
     * @param result array containing distance and its standard deviation, in such
     *               order.
     * @param <P> a {@link Point} type.
     */
    private static <P extends Point> void computeDistanceAndStandardDeviationRssi(
            RadioSourceLocated<P> locatedSource, double rxPower,
            Double rxPowerStandardDeviation,
            Double positionStandardDeviation,
            Double[] result) {
        if (!(locatedSource instanceof RadioSourceWithPower)) {
            return;
        }

        RadioSourceWithPower poweredSource = (RadioSourceWithPower)locatedSource;

        //source related parameters
        //transmitted power in dBm's
        double txPower = poweredSource.getTransmittedPower();
        Double txPowerStandardDeviation = poweredSource.getTransmittedPowerStandardDeviation();

        //path loss exponent
        double pathLossExponent = poweredSource.getPathLossExponent();
        Double pathLossExponentStandardDeviation = poweredSource.getPathLossExponentStandardDeviation();

        //TODO: take into account covariance between tx power and path loss exponent

        double frequency = poweredSource.getFrequency();

        double txPowerVariance = txPowerStandardDeviation != null ?
                txPowerStandardDeviation * txPowerStandardDeviation : 0.0;
        double rxPowerVariance = rxPowerStandardDeviation != null ?
                rxPowerStandardDeviation * rxPowerStandardDeviation : 0.0;
        double pathLossVariance = pathLossExponentStandardDeviation != null ?
                pathLossExponentStandardDeviation * pathLossExponentStandardDeviation : 0.0;

        double distanceVariance = 0.0;
        try {
            MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPower, rxPower, pathLossExponent, frequency, txPowerVariance,
                    rxPowerVariance, pathLossVariance);
            //distance
            result[0] = dist.getMean()[0];

            //distance variance
            distanceVariance = dist.getCovariance().getElementAt(0, 0);
        } catch (IndoorException e) {
            result[0] = computeDistanceRssi(locatedSource, rxPower);
            if (rxPowerStandardDeviation != null) {
                //take into account only received power standard deviation
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
