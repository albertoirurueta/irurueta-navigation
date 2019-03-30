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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        double[] sourceQualityScores = new double[numSources];
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
        }

        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        double[] readingsQualityScores = new double[numReadings];
        for (int i = 0; i < numReadings; i++) {
            double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            readings.add(new RssiReading<WifiAccessPoint>(sources.get(0), rssi));
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
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
        } catch (IllegalArgumentException ignore) { }
        try {
            sorter = new ReadingSorter<>(
                    sources, fingerprint, sourceQualityScores, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(sorter);
    }

    @Test
    public void testSortSameSourceRangingReadingsDifferentQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        for (int j = 0; j < numReadings; j++) {
            ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    public void testSortSameSourceRangingAndRssiReadingsDifferentQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        for (int j = 0; j < numReadings; j++) {
            ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertTrue(previousQuality >= readingWithQualityScore.qualityScore);
            previousQuality = readingWithQualityScore.qualityScore;
        }
    }

    @Test
    public void testSortSameSourceRssiReadingsDifferentQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[3 * numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
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

        Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        double previousQuality = Double.MAX_VALUE;
        ReadingType previousType = ReadingType.RANGING_READING;
        for (k = 0; k < 3 * numReadings; k++) {
            ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(k);

            ReadingType type = readingWithQualityScore.reading.getType();
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        for (int j = 0; j < numReadings; j++) {
            ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    public void testSortSameSourceRangingAndRssiReadingsSameQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        for (int j = 0; j < numReadings; j++) {
            ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    public void testSortSameSourceRssiReadingsSameQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        for (int j = 0; j < numReadings; j++) {
            ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(j);
            assertSame(readingWithQualityScore.reading, readings.get(j));
        }
    }

    @Test
    public void testSortSameSourceMixedReadingsSameQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = 1;
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[3 * numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
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

        Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), 1);

        ReadingType previousType = ReadingType.RANGING_READING;
        for (k = 0; k < 3 * numReadings; k++) {
            ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                    result.get(0).readingsWithQualityScores.get(k);

            ReadingType type = readingWithQualityScore.reading.getType();
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingAndRssiReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsDifferentSourceQualityScoresDifferentReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[3 * numSources * numReadings];

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
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

        Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            double previousReadingQuality = Double.MAX_VALUE;
            ReadingType previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(k);

                ReadingType type = readingWithQualityScore.reading.getType();
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingAndRssiReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsDifferentSourceQualityScoresSameReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[3 * numSources * numReadings];
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = randomizer.nextDouble();

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
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

        Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        double previousSourceQuality = Double.MAX_VALUE;
        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertTrue(previousSourceQuality >= sourceWithQualityScore.qualityScore);
            previousSourceQuality = sourceWithQualityScore.qualityScore;

            ReadingType previousType = ReadingType.RANGING_READING;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);

                ReadingType type = readingWithQualityScore.reading.getType();
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRangingAndRssiReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = randomizer.nextDouble();
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertTrue(previousReadingQuality >= readingWithQualityScore.qualityScore);
                previousReadingQuality = readingWithQualityScore.qualityScore;
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsSameSourceQualityScoresDifferentReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[3 * numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
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

        Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            double previousReadingQuality = Double.MAX_VALUE;
            ReadingType previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(k);

                ReadingType type = readingWithQualityScore.reading.getType();
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
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                readings.add(new RangingReading<WifiAccessPoint>(accessPoint,
                        distance));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RangingFingerprint<WifiAccessPoint, RangingReading<WifiAccessPoint>> fingerprint =
                new RangingFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRanginAndRssiReadingsSameSourceQualityScoresSameReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RangingAndRssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RangingAndRssiReading<WifiAccessPoint>(accessPoint,
                        distance, rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RangingAndRssiFingerprint<WifiAccessPoint, RangingAndRssiReading<WifiAccessPoint>> fingerprint =
                new RangingAndRssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RangingAndRssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RangingAndRssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RangingAndRssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesRssiReadingsSameSourceQualityScoresSameReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                readings.add(new RssiReading<WifiAccessPoint>(accessPoint,
                        rssi));

                readingsQualityScores[k] = readingQualityScoreValue;
                k++;
            }
        }

        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, RssiReading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, RssiReading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            for (int j = 0; j < numReadings; j++) {
                ReadingSorter.ReadingWithQualityScore<RssiReading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(j);
                assertEquals(readingQualityScoreValue,
                        readingWithQualityScore.qualityScore, 0.0);
            }
        }
    }

    @Test
    public void testSortMultipleSourcesMixedReadingsSameSourceQualityScoresSameReadingQualityScores() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
        int numReadings = randomizer.nextInt(MIN_READINGS, MAX_READINGS);

        double[] sourceQualityScores = new double[numSources];
        double[] readingsQualityScores = new double[3 * numSources * numReadings];
        double sourceQualityScoreValue = randomizer.nextDouble();
        double readingQualityScoreValue = randomizer.nextDouble();

        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        List<Reading<WifiAccessPoint>> readings = new ArrayList<>();
        int k = 0;
        for (int i = 0; i < numSources; i++) {
            WifiAccessPointLocated2D accessPoint = new WifiAccessPointLocated2D(
                    "id" + i, FREQUENCY, new InhomogeneousPoint2D());
            sources.add(accessPoint);
            sourceQualityScores[i] = sourceQualityScoreValue;

            for (int j = 0; j < numReadings; j++) {
                double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);
                double rssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
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

        Fingerprint<WifiAccessPoint, Reading<WifiAccessPoint>> fingerprint =
                new Fingerprint<>();
        fingerprint.setReadings(readings);

        ReadingSorter<Point2D, Reading<WifiAccessPoint>> sorter = new ReadingSorter<>(
                sources, fingerprint, sourceQualityScores, readingsQualityScores);

        sorter.sort();

        // check order
        List<ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>>> result =
                sorter.getSortedSourcesAndReadings();

        assertEquals(result.size(), numSources);

        for (int i = 0; i < numSources; i++) {
            ReadingSorter.RadioSourceSourceWithQualityScore<Point2D, Reading<WifiAccessPoint>> sourceWithQualityScore =
                    result.get(i);
            assertSame(sourceWithQualityScore.source, sources.get(i));
            assertEquals(sourceWithQualityScore.qualityScore, sourceQualityScoreValue,
                    0.0);

            ReadingType previousType = ReadingType.RANGING_READING;
            for (k = 0; k < 3 * numReadings; k++) {
                ReadingSorter.ReadingWithQualityScore<Reading<WifiAccessPoint>> readingWithQualityScore =
                        sourceWithQualityScore.readingsWithQualityScores.get(k);

                ReadingType type = readingWithQualityScore.reading.getType();
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
