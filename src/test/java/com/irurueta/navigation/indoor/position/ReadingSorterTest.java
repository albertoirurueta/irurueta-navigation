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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class ReadingSorterTest {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final int MIN_READINGS = 6;
    private static final int MAX_READINGS = 20;

    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 10.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    @Test
    public void testConstructor() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final double[] sourceQualityScores = new double[numSources];
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
        }

        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        final double[] readingsQualityScores = new double[numReadings];
        for (int i = 0; i < numReadings; i++) {
            final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            readings.add(new RssiReading<WifiAccessPoint>(sources.get(0), rssi));
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        assertSame(sorter.getSources(), sources);
        assertSame(sorter.getFingerprint(), fingerprint);
        assertSame(sorter.getSourceQualityScores(), sourceQualityScores);
        assertSame(sorter.getFingerprintReadingsQualityScores(), readingsQualityScores);
        assertNull(sorter.getSortedSourcesAndReadings());

        // force IllegalArgumentException
        sorter = null;
        try {
            sorter = new ReadingSorter<>(
                    sources, fingerprint, new double[1], readingsQualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        try {
            sorter = new ReadingSorter<>(
                    sources, fingerprint, sourceQualityScores, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
        assertNull(sorter);
    }

    @Test
    public void testSortSameSourceRangingReadingsDifferentQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        for (int j = 0; j < numReadings; j++) {
            final ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    public void testSortSameSourceRangingAndRssiReadingsDifferentQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        for (int j = 0; j < numReadings; j++) {
            final ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    public void testSortSameSourceRssiReadingsDifferentQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        for (int j = 0; j < numReadings; j++) {
            ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    public void testSortSameSourceMixedReadingsDifferentQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[3 * numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        ReadingType previousType = ReadingType.RANGING_READING;
        for (k = 0; k < 3 * numReadings; k++) {
            final ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(k);

            final ReadingType type = readingWithQualityScore.reading.getType();
            if (type != previousType) {
                // check correct order of type changes
                if (type == ReadingType.RANGING_AND_RSSI_READING) {
                    assertEquals(previousType, ReadingType.RANGING_READING);
                }
                if (type == ReadingType.RSSI_READING) {
                    assertEquals(previousType, ReadingType.RANGING_AND_RSSI_READING);
                }

                // when type changes reset quality score
                previousQuality = Double.MAX_VALUE;
                previousType = readingWithQualityScore.reading.getType();
            }

            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    public void testSortSameSourceRangingReadingsSameQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        for (int j = 0; j < numReadings; j++) {
            final ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    public void testSortSameSourceRangingAndRssiReadingsSameQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        for (int j = 0; j < numReadings; j++) {
            final ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    public void testSortSameSourceRssiReadingsSameQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        for (int j = 0; j < numReadings; j++) {
            final ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    public void testSortSameSourceMixedReadingsSameQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = 1;
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[3 * numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        ReadingType previousType = ReadingType.RANGING_READING;
        for (k = 0; k < 3 * numReadings; k++) {
            final ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(k);

            final ReadingType type = readingWithQualityScore.reading.getType();
            if (type != previousType) {
                // check correct order of type changes
                if (type == ReadingType.RANGING_AND_RSSI_READING) {
                    assertEquals(previousType, ReadingType.RANGING_READING);
                }
                if (type == ReadingType.RSSI_READING) {
                    assertEquals(previousType, ReadingType.RANGING_AND_RSSI_READING);
                }

                // when type changes reset quality score
                previousType = readingWithQualityScore.reading.getType();
            }

            assertEquals(readingQualityScoreValue, readingWithQualityScore.qualityScore,
                    0.0);
        }
    }

    @Test
    public void testSortMultipleSourcesRangingReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingAndRssiReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[3 * numSources * numReadings];

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            ReadingType previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                final ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(k);

                final ReadingType type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_READING);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_AND_RSSI_READING);
                    }

                    // when type changes reset quality score
                    previousReadingQuality = Double.MAX_VALUE;
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingAndRssiReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[3 * numSources * numReadings];
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            ReadingType previousType = ReadingType.RANGING_READING;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);

                final ReadingType type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_READING);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_AND_RSSI_READING);
                    }

                    // when type changes reset quality score
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingAndRssiReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[3 * numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            ReadingType previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                final ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(k);

                final ReadingType type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_READING);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_AND_RSSI_READING);
                    }

                    // when type changes reset quality score
                    previousReadingQuality = Double.MAX_VALUE;
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRanginReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRanginAndRssiReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            for (int j = 0; j < numReadings; j++) {
                final ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsSameSourceQualityScoresSameReadingQualityScores() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        final int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        final double[] sourceQualityScores = new double[numSources];
        final double[] readingsQualityScores = new double[3 * numSources * numReadings];
        final double sourceQualityScoreValue = randomizer.nextDouble();
        final double readingQualityScoreValue = randomizer.nextDouble();

        final List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        final List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            final WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                final double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        final Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        final ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        final List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            final ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            ReadingType previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                final ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(k);

                final ReadingType type = readingWithQualityScore.reading.getType();
                if (type != previousType) {
                    // check correct order of type changes
                    if (type == ReadingType.RANGING_AND_RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_READING);
                    }
                    if (type == ReadingType.RSSI_READING) {
                        assertEquals(previousType, ReadingType.RANGING_AND_RSSI_READING);
                    }

                    // when type changes reset quality score
                    previousType = readingWithQualityScore.reading.getType();
                }

                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }
}
