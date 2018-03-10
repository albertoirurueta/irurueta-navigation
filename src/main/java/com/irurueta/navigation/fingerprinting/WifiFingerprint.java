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
package com.irurueta.navigation.fingerprinting;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains WiFi readings for an unknown location to be determined.
 */
@SuppressWarnings("WeakerAccess")
public class WifiFingerprint implements Serializable {

    /**
     * Non-located WiFi readings.
     */
    private List<WifiRssiReading> mReadings = new ArrayList<>();

    /**
     * Constructor.
     */
    public WifiFingerprint() { }

    /**
     * Constructor.
     * @param readings non-located WiFi readings.
     * @throws IllegalArgumentException if provided readings is null.
     */
    public WifiFingerprint(List<WifiRssiReading> readings)
            throws IllegalArgumentException {
        if (readings == null) {
            throw new IllegalArgumentException();
        }
        mReadings = readings;
    }

    /**
     * Gets non-located WiFi readings.
     * @return non-located WiFi readings.
     */
    public List<WifiRssiReading> getReadings() {
        return mReadings;
    }

    /**
     * Sets non-located WiFi readings.
     * @param readings non-located WiFi readings.
     * @throws IllegalArgumentException if provided readings is null.
     */
    public void setReadings(List<WifiRssiReading> readings)
            throws IllegalArgumentException {
        if (readings == null) {
            throw new IllegalArgumentException();
        }
        mReadings = readings;
    }

    /**
     * Gets euclidean distance of signal readings from another fingerprint.
     * @param otherFingerprint other fingerprint to compare.
     * @return euclidean distance of signal readings from another fingerprint.
     */
    public double distanceTo(WifiFingerprint otherFingerprint) {
        return Math.sqrt(sqrDistanceTo(otherFingerprint));
    }

    /**
     * Gets squared euclidean distance of signal readings from another fingerprint.
     * @param otherFingerprint other fingerprint to compare.
     * @return squared euclidean distance of signal readings from another
     * fingerprint.
     */
    public double sqrDistanceTo(WifiFingerprint otherFingerprint) {
        if (otherFingerprint == null) {
            return Double.MAX_VALUE;
        }

        List<WifiRssiReading> otherReadings = otherFingerprint.getReadings();
        int numAccessPoints = 0;
        double result = 0.0, diff;
        for (WifiRssiReading reading : mReadings) {
            for (WifiRssiReading otherReading : otherReadings) {
                if (reading.hasSameAccessPoint(otherReading)) {
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
}
