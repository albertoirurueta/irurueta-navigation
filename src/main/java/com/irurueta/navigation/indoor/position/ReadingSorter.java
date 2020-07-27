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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.indoor.*;

import java.util.*;

/**
 * Evenly sorts readings of a fingerprint among different radio sources taking into
 * account their respective quality scores as well.
 *
 * @param <P> a {@link Point} type.
 * @param <R> a {@link Reading} type.
 */
public class ReadingSorter<P extends Point<?>, R extends Reading<? extends RadioSource>> {

    /**
     * Sources to be taken into account within readings.
     */
    private final List<? extends RadioSourceLocated<P>> mSources;

    /**
     * Fingerprint containing readings of different sources.
     */
    private final Fingerprint<? extends RadioSource, ? extends R> mFingerprint;

    /**
     * Quality scores associated to each radio source.
     */
    private final double[] mSourceQualityScores;

    /**
     * Quality scores associated to each reading within the fingerprint.
     */
    private final double[] mFingerprintReadingsQualityScores;

    /**
     * Contains sorted sources and readings.
     */
    private List<RadioSourceSourceWithQualityScore<P, R>> mSortedSourcesAndReadings;

    /**
     * Constructor.
     *
     * @param sources                          sources to take into account within
     *                                         readings.
     * @param fingerprint                      fingerprint containing readings of
     *                                         different sources.
     * @param sourceQualityScores              quality scores associated to each radio
     *                                         source.
     * @param fingerprintReadingsQualityScores quality scores associated to each reading
     *                                         within the fingerprint.
     * @throws IllegalArgumentException if number of source quality scores is not equal
     *                                  to the number of sources, or if number of
     *                                  fingerprint reading quality scores is not equal
     *                                  to the number of readings within fingerprint.
     */
    ReadingSorter(final List<? extends RadioSourceLocated<P>> sources,
                  final Fingerprint<? extends RadioSource, ? extends R> fingerprint,
                  final double[] sourceQualityScores,
                  final double[] fingerprintReadingsQualityScores) {
        if (sources.size() != sourceQualityScores.length) {
            throw new IllegalArgumentException();
        }

        if (fingerprint.getReadings().size() != fingerprintReadingsQualityScores.length) {
            throw new IllegalArgumentException();
        }

        mSources = sources;
        mFingerprint = fingerprint;
        mSourceQualityScores = sourceQualityScores;
        mFingerprintReadingsQualityScores = fingerprintReadingsQualityScores;
    }

    /**
     * Gets sources to be taken into account within readings.
     *
     * @return sources to be taken into account within readings.
     */
    public List<? extends RadioSourceLocated<P>> getSources() {
        return mSources;
    }

    /**
     * Gets fingerprint containing readings of different sources.
     *
     * @return fingerprint containing readings of different sources.
     */
    public Fingerprint<? extends RadioSource, ? extends R> getFingerprint() {
        return mFingerprint;
    }

    /**
     * Gets quality scores associated to each radio source.
     *
     * @return quality scores associated to each radio source.
     */
    double[] getSourceQualityScores() {
        return mSourceQualityScores;
    }

    /**
     * Gets quality scores associated to each reading within the fingerprint.
     *
     * @return quality scores associated to each reading within the fingerprint.
     */
    double[] getFingerprintReadingsQualityScores() {
        return mFingerprintReadingsQualityScores;
    }

    /**
     * Sorts readings with quality scores.
     */
    void sort() {

        final Map<RadioSourceLocated<P>, RadioSourceSourceWithQualityScore<P, R>> sourcesMap = new HashMap<>();

        // build sources
        final List<RadioSourceSourceWithQualityScore<P, R>> sourcesWithQualityScores = new ArrayList<>();
        int sourcePosition = 0;
        for (final RadioSourceLocated<P> source : mSources) {
            final RadioSourceSourceWithQualityScore<P, R> sourceWithQualityScore =
                    new RadioSourceSourceWithQualityScore<>();
            sourceWithQualityScore.source = source;
            sourceWithQualityScore.qualityScore = mSourceQualityScores[sourcePosition];
            sourceWithQualityScore.position = sourcePosition;
            sourceWithQualityScore.readingsWithQualityScores = new ArrayList<>();

            sourcesWithQualityScores.add(sourceWithQualityScore);

            sourcesMap.put(source, sourceWithQualityScore);

            sourcePosition++;
        }

        // build readings
        int readingPosition = 0;
        for (@SuppressWarnings("unchecked") final R reading : mFingerprint.getReadings()) {
            // noinspection all
            final RadioSourceSourceWithQualityScore<P, R> sourceWithQualityScore =
                    sourcesMap.get(reading.getSource());
            if (sourceWithQualityScore == null) {
                continue;
            }

            final List<ReadingWithQualityScore<R>> readingsWithQualityScores =
                    sourceWithQualityScore.readingsWithQualityScores;

            final ReadingWithQualityScore<R> readingWithQualityScore = new ReadingWithQualityScore<>();
            readingWithQualityScore.reading = reading;
            readingWithQualityScore.qualityScore = mFingerprintReadingsQualityScores[readingPosition];
            readingWithQualityScore.position = readingPosition;

            readingsWithQualityScores.add(readingWithQualityScore);

            readingPosition++;
        }

        // sort all readings within sources
        for (final RadioSourceSourceWithQualityScore<P, R> sourceWithQualityScore : sourcesWithQualityScores) {
            // sort all readings for this source from highest to lowest quality

            // noinspection unchecked
            final ReadingWithQualityScore<R>[] readingsWithQualityScoresArray =
                    new ReadingWithQualityScore[sourceWithQualityScore.readingsWithQualityScores.size()];
            sourceWithQualityScore.readingsWithQualityScores.toArray(readingsWithQualityScoresArray);
            Arrays.sort(readingsWithQualityScoresArray, new ReadingComparator<R>());

            sourceWithQualityScore.readingsWithQualityScores = Arrays.asList(
                    readingsWithQualityScoresArray);
        }

        // sort all sources from highest to lowest quality

        // noinspection unchecked
        final RadioSourceSourceWithQualityScore<P, R>[] sourcesWithQualityScoresArray =
                new RadioSourceSourceWithQualityScore[sourcesWithQualityScores.size()];
        sourcesWithQualityScores.toArray(sourcesWithQualityScoresArray);
        Arrays.sort(sourcesWithQualityScoresArray, new RadioSourceComparator<P, R>());

        mSortedSourcesAndReadings = Arrays.asList(sourcesWithQualityScoresArray);
    }

    /**
     * Gets sorted sources and readings.
     *
     * @return sorted sources and readings.
     */
    List<RadioSourceSourceWithQualityScore<P, R>> getSortedSourcesAndReadings() {
        return mSortedSourcesAndReadings;
    }

    /**
     * Contains a reading with its associated quality score.
     *
     * @param <R> a {@link Reading} type.
     */
    static class ReadingWithQualityScore<R extends Reading<? extends RadioSource>> {
        /**
         * A reading.
         */
        R reading;

        /**
         * Reading quality score.
         */
        double qualityScore;

        /**
         * Original position.
         */
        int position;
    }

    /**
     * Contains a radio source with its associated quality score.
     *
     * @param <P> a {@link Point} type.
     * @param <R> a {@link Reading} type.
     */
    static class RadioSourceSourceWithQualityScore<P extends Point<?>,
            R extends Reading<? extends RadioSource>> {
        /**
         * Radio source.
         */
        RadioSourceLocated<P> source;

        /**
         * Radio source quality score.
         */
        double qualityScore;

        /**
         * Original position.
         */
        int position;

        List<ReadingWithQualityScore<R>> readingsWithQualityScores;
    }

    /**
     * Comparator to order readings based on their quality scores.
     *
     * @param <R> a {@link Reading} type.
     */
    private static class ReadingComparator<R extends Reading<? extends RadioSource>> implements
            Comparator<ReadingWithQualityScore<R>> {

        /**
         * Orders readings so that readings with higher scores go first.
         *
         * @param o1 1st item to be compared.
         * @param o2 2nd item to be compared.
         * @return -1 if first item goes first, 0 if both are equal and 1 if first item
         * goes second.
         */
        @Override
        public int compare(final ReadingWithQualityScore<R> o1, final ReadingWithQualityScore<R> o2) {

            // Take reading type into account so that ranging readings go first,
            // then ranging+RSSI and finally RSSI readings.
            if (o1.reading.getType() != o2.reading.getType()) {
                switch (o1.reading.getType()) {
                    case RANGING_READING:
                        // o1 goes first (o2 last)
                        return -1;

                    case RANGING_AND_RSSI_READING:
                        switch (o2.reading.getType()) {
                            case RANGING_READING:
                                // o2 goes first (o1 last)
                                return 1;
                            case RSSI_READING:
                                // o1 goes first (o2 last)
                                return -1;
                        }
                        break;
                    case RSSI_READING:
                        // o2 goes first (o1 last)
                        return 1;
                }
            }


            // same reading type

            // order by reading quality score
            return Double.compare(o2.qualityScore, o1.qualityScore);
        }
    }

    /**
     * Comparator to order radio sources based on their quality scores.
     *
     * @param <P> a {@link Point} type.
     * @param <R> a {@link Reading} type.
     */
    private static class RadioSourceComparator<P extends Point<?>,
            R extends Reading<? extends RadioSource>> implements
            Comparator<RadioSourceSourceWithQualityScore<P, R>> {

        /**
         * Orders radio sources so that radio sources with higher scores go first.
         *
         * @param o1 1st item to be compared.
         * @param o2 2nd item to be compared.
         * @return -1 if first item goes first, 0 if both are equal and 1 if first item
         * goes second.
         */
        @Override
        public int compare(final RadioSourceSourceWithQualityScore<P, R> o1,
                           final RadioSourceSourceWithQualityScore<P, R> o2) {
            return Double.compare(o2.qualityScore, o1.qualityScore);
        }
    }
}
