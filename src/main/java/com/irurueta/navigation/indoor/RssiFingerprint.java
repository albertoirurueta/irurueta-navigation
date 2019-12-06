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
package com.irurueta.navigation.indoor;

import java.util.List;

/**
 * Contains RSSI readings from several radio sources for an unknown location to be
 * determined.
 * @param <R> a {@link RssiReading} type.
 * @param <S> a {@link RadioSource} type.
 */
@SuppressWarnings("WeakerAccess")
public class RssiFingerprint<S extends RadioSource, R extends RssiReading<S>>
        extends Fingerprint<S, R> {

    /**
     * Constructor.
     */
    public RssiFingerprint() { }

    /**
     * Constructor.
     * @param readings non-located RSSI readings.
     * @throws IllegalArgumentException if provided readings is null.
     */
    public RssiFingerprint(List<R> readings) {
        super(readings);
    }

    /**
     * Gets euclidean distance of signal readings from another fingerprint.
     * @param otherFingerprint other fingerprint to compare.
     * @return euclidean distance of signal readings from another fingerprint.
     */
    public double distanceTo(RssiFingerprint<S, R> otherFingerprint) {
        return Math.sqrt(sqrDistanceTo(otherFingerprint));
    }

    /**
     * Gets squared euclidean distance of signal readings from another fingerprint.
     * @param otherFingerprint other fingerprint to compare.
     * @return squared euclidean distance of signal readings from another
     * fingerprint.
     */
    @SuppressWarnings("Duplicates")
    public double sqrDistanceTo(RssiFingerprint<S, R> otherFingerprint) {
        if (otherFingerprint == null) {
            return Double.MAX_VALUE;
        }

        List<R> otherReadings = otherFingerprint.getReadings();
        int numAccessPoints = 0;
        double result = 0.0;
        double diff;
        for (R reading : mReadings) {
            for (R otherReading : otherReadings) {
                if (reading.hasSameSource(otherReading)) {
                    diff = reading.getRssi() - otherReading.getRssi();
                    result += diff * diff;
                    numAccessPoints++;
                }
            }
        }

        if (numAccessPoints == 0) {
            return Double.MAX_VALUE;
        }

        return result;
    }

    /**
     * Gets average RSSI (received signal strength indicator) of all readings contained in this fingerprint
     * expressed in dB's.
     * @return average RSSI of all readings.
     */
    public double getMeanRssi() {
        if (mReadings == null || mReadings.isEmpty()) {
            return Double.MAX_VALUE;
        }

        double result = 0.0;
        for (R reading : mReadings) {
            result += reading.getRssi() / (double)mReadings.size();
        }

        return result;
    }

    /**
     * Gets euclidean distance of signal readings from another fingerprint.
     * @param otherFingerprint other fingerprint to compare.
     * @return euclidean distance of signal readings from another fingerprint.
     */
    public double noMeanDistanceTo(RssiFingerprint<S, R> otherFingerprint) {
        return Math.sqrt(noMeanSqrDistanceTo(otherFingerprint));
    }

    /**
     * Gets squared euclidean distance of signal readings with mean RSSI removed from another fingerprint.
     * Mean RSSI's are taken into account so that bias effects introduced by different device's hardware is
     * partially removed.
     * @param otherFingerprint other fingerprint to compare.
     * @return squared euclidean distance of signal readings from another
     * fingerprint with average RSSI's removed.
     */
    @SuppressWarnings("Duplicates")
    public double noMeanSqrDistanceTo(RssiFingerprint<S, R> otherFingerprint) {
        if (otherFingerprint == null) {
            return Double.MAX_VALUE;
        }

        List<R> otherReadings = otherFingerprint.getReadings();
        int numAccessPoints = 0;
        double avgRssiThis = 0.0;
        double avgRssiOther = 0.0;
        for (R reading : mReadings) {
            for (R otherReading : otherReadings) {
                if (reading.hasSameSource(otherReading)) {
                    avgRssiThis += reading.getRssi();
                    avgRssiOther += otherReading.getRssi();

                    numAccessPoints++;
                }
            }

        }

        if (numAccessPoints == 0) {
            return Double.MAX_VALUE;
        }

        avgRssiThis /= numAccessPoints;
        avgRssiOther /= numAccessPoints;


        double result = 0.0;
        double diff;
        for (R reading : mReadings) {
            for (R otherReading : otherReadings) {
                if (reading.hasSameSource(otherReading)) {
                    diff = (reading.getRssi() - avgRssiThis) -
                            (otherReading.getRssi() - avgRssiOther);
                    result += diff * diff;
                }
            }
        }

        return result;
    }
}
