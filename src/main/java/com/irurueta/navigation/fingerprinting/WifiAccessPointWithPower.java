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

/**
 * Data related to a WiFi access point with estimated transmitted power.
 */
@SuppressWarnings("WeakerAccess")
public class WifiAccessPointWithPower extends WifiAccessPoint {

    /**
     * Transmitted power expressed in dBm's.
     */
    private double mTransmittedPower;

    /**
     * Standard deviation of transmitted power value.
     */
    private double mTransmittedPowerStandardDeviation;

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPower(String bssid, double frequency,
            double transmittedPower) throws IllegalArgumentException {
        super(bssid, frequency);
        mTransmittedPower = transmittedPower;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPointWithPower(String bssid, double frequency, String ssid,
            double transmittedPower) throws IllegalArgumentException {
        super(bssid, frequency, ssid);
        mTransmittedPower = transmittedPower;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param transmittedPower transmitted power by this access point (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(String bssid, double frequency,
            double transmittedPower, double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        this(bssid, frequency, transmittedPower);

        if (transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @param transmittedPower transmitted power by this access point (expressed in dBM's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @throws IllegalArgumentException if either BSSID is null, frequency is negative,
     * or transmitted power standard deviation is negative.
     */
    public WifiAccessPointWithPower(String bssid, double frequency, String ssid,
            double transmittedPower, double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        this(bssid, frequency, ssid, transmittedPower);

        if (transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPointWithPower() {
        super();
    }

    /**
     * Gets transmitted power expressed in dBm's.
     * @return transmitted power expressed in dBm's.
     */
    public double getTransmittedPower() {
        return mTransmittedPower;
    }

    /**
     * Gets standard deviation of transmitted power value.
     * @return standard deviation of transmitted power value.
     */
    public double getTransmittedPowerStandardDeviation() {
        return mTransmittedPowerStandardDeviation;
    }
}
