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

import java.io.Serializable;

/**
 * Data related to a WiFi access point.
 */
@SuppressWarnings("WeakerAccess")
public class WifiAccessPoint implements Serializable, RadioSource {

    /**
     * Basic service set identifier of this access point in the form of a six-byte MAC address:
     * xx:xx:xx:xx:xx:xx.
     */
    private String mBssid;

    /**
     * Frequency used by this Access Point (expressed in Hz).
     */
    private double mFrequency;

    /**
     * Service set identifier (SSID) of this 802.11 network. This value is optional.
     */
    private String mSsid;

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPoint(String bssid, double frequency) throws IllegalArgumentException {
        if (bssid == null) {
            throw new IllegalArgumentException();
        }

        if (frequency < 0.0) {
            throw new IllegalArgumentException();
        }

        mBssid = bssid;
        mFrequency = frequency;
    }

    /**
     * Constructor.
     * @param bssid basic service set identifier of this access point in the form of a six-byte MAC address:
     *              xx:xx:xx:xx:xx:xx.
     * @param frequency frequency used by this Access Point (expressed in Hz).
     * @param ssid service set identifier (SSID) of this 802.11 network.
     * @throws IllegalArgumentException if BSSID is null or frequency is negative.
     */
    public WifiAccessPoint(String bssid, double frequency, String ssid) throws IllegalArgumentException {
        this(bssid, frequency);
        mSsid = ssid;
    }

    /**
     * Empty constructor.
     */
    protected WifiAccessPoint() { }

    /**
     * Gets the basic service set identifier of this access point in the form of a six-byte MAC address:
     * xx:xx:xx:xx:xx:xx.
     * @return the basic service set identifier.
     */
    public String getBssid() {
        return mBssid;
    }

    /**
     * Gets frequency used by this Access Point (expressed in Hz).
     * @return frequency used by this Access Point (expressed in Hz).
     */
    @Override
    public double getFrequency() {
        return mFrequency;
    }

    /**
     * Gets service set identifier (SSID) of this 802.11 network.
     * @return service set identifier (SSID).
     */
    public String getSsid() {
        return mSsid;
    }

    /**
     * Indicates whether two access points are considered to be equal or not.
     * Two access points are considered equal if they have the same BSSID.
     * @param obj other object to be compared.
     * @return true if both access points are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }

        if (!(obj instanceof WifiAccessPoint)) {
            return false;
        }

        WifiAccessPoint other = (WifiAccessPoint)obj;
        return mBssid.equals(other.mBssid);
    }

    /**
     * Returns hashcode associated to this access point.
     * @return hashcode associated to this access point.
     */
    @Override
    public int hashCode() {
        return mBssid.hashCode();
    }

    /**
     * Gets radio source type, which can be either a WiFi Access point or a bluetooth Beacon.
     * @return radio source type.
     */
    @Override
    public RadioSourceType getType() {
        return RadioSourceType.WIFI_ACCESS_POINT;
    }
}
