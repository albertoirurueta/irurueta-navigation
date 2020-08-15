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

/**
 * Utility class to handle Android's AltBeacon library coefficients used te determine distance for calibrated devices.
 * AltBeacon uses the following formula to estimate Beacon's distance:
 * d = coefficient1 * ratio^coefficient2 + coefficient3
 * <p>
 * Considered that radio signal propagation follows equation:
 * Pr = Pt * k / d^n,
 * where:
 * - Pr is received signal strength (expressed in mW),
 * - Pt is transmitted signal strength (expressed in mW),
 * - k is equal to k = c / (4 * pi * f), where:
 * - c is speed of light (which is aproximately 9*10^8 m/s)
 * - f is the radio link frequency used by the beacon.
 * <p>
 * Hence, considering:
 * ratio = Pr / Pt
 * <p>
 * which is equal to:
 * ratio = k / d^n
 * <p>
 * And isolating d, whe obtain that:
 * d^n = k / ratio
 * <p>
 * d = (k / ratio)^(1/n)
 * d = k^(1/n)*ratio^(-1/n)
 * <p>
 * And thus we can see from this last expression that:
 * coefficient1 = k^(1/n) = (c / (4 * pi * f))^(1/n)
 * coefficient2 = -1/n, where n is the path loss exponent and depends on the
 * environment where the device is used (typically on free space n is equal to
 * 2.0, but can usually range from 1.6 to 1.8 on indoor environments)
 * coefficient3 is a bias term calibrated for each device.
 */
@SuppressWarnings("WeakerAccess")
public class AltBeaconUtils {

    /**
     * Speed of light expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = 299792458.0;

    /**
     * Constructor to prevent instantiation of utility class.
     */
    private AltBeaconUtils() {
    }

    /**
     * Gets k value using provided frequency.
     * K is defined as k = c / (4 * pi * f), where:
     * - c is the speed of light expressed in m/s.
     * - f is the frequency expressed in Hz.
     *
     * @param frequency a frequency (expressed in Hz).
     * @return k value.
     */
    public static double getK(final double frequency) {
        //k = c / (4 * pi * f)
        return SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
    }

    /**
     * Gets k value using AltBeacon's coefficients.
     *
     * @param coefficient1 1st coefficient.
     * @param coefficient2 2nd coefficient.
     * @return k value.
     */
    public static double getK(
            final double coefficient1, final double coefficient2) {
        //coefficient1 = k^(1/n), hence
        //k = coefficient1^n
        final double n = -1.0 / coefficient2;
        return Math.pow(coefficient1, n);
    }

    /**
     * Gets frequency from k value.
     *
     * @param k k value.
     * @return frequency expressed in Hz.
     */
    public static double getFrequency(final double k) {
        //k = c / (4*pi*f) --> f = c / (4*pi*k)
        return SPEED_OF_LIGHT / (4.0 * Math.PI * k);
    }

    /**
     * Gets frequency using AltBeacon's coefficients.
     *
     * @param coefficient1 1st coefficient.
     * @param coefficient2 2nd coefficient.
     * @return frequency expressed in Hz.
     */
    public static double getFrequency(
            final double coefficient1, final double coefficient2) {
        return getFrequency(getK(coefficient1, coefficient2));
    }

    /**
     * Gets path loss exponent using AltBeacon's coefficient.
     *
     * @param coefficient2 2nd coefficient.
     * @return path loss exponent.
     */
    public static double getPathLossExponent(final double coefficient2) {
        //coefficient2 = -1/n
        return -1.0 / coefficient2;
    }

    /**
     * Gets distance using AltBeacon's coefficients.
     *
     * @param coefficient1 1st coefficient.
     * @param coefficient2 2nd coefficient.
     * @param coefficient3 3rd coefficient.
     * @param ratio        ratio between received power and transmitted power.
     * @return distance expressed in meters.
     */
    public static double getDistance(
            final double coefficient1, final double coefficient2,
            final double coefficient3, final double ratio) {
        //d = coefficient1 * ratio^coefficient2 + coefficient3
        return coefficient1 * Math.pow(ratio, coefficient2) + coefficient3;
    }

    /**
     * Gets distance using AltBeacon's coefficients.
     *
     * @param coefficient1     1st coefficient.
     * @param coefficient2     2nd coefficient.
     * @param coefficient3     3rd coefficient.
     * @param receivedPower    received power (expressed in mW).
     * @param transmittedPower transmitted power (expressed in mW).
     * @return distance expressed in meters.
     */
    public static double getDistance(
            final double coefficient1, final double coefficient2,
            final double coefficient3, final double receivedPower,
            final double transmittedPower) {
        return getDistance(coefficient1, coefficient2, coefficient3,
                receivedPower / transmittedPower);
    }

    /**
     * Gets power ratio between received power and transmitted power
     * using AltBeacon's coefficients.
     *
     * @param coefficient1 1st coefficient.
     * @param coefficient2 2nd coefficient.
     * @param coefficient3 3rd coefficient.
     * @param distance     distance expressed in meters.
     * @return power ratio.
     */
    public static double getRatio(
            final double coefficient1, final double coefficient2,
            final double coefficient3, final double distance) {
        //d = coefficient1 * ratio^coefficient2 + coefficient3
        //d - coefficient3 = coefficient1 * ratio^coefficient2
        //(d - coefficient3) / coefficient1 = ratio^coefficient2
        //ratio = ((d - coefficient3) / coefficient1)^(1/coefficient2)
        return Math.pow((distance - coefficient3) / coefficient1,
                1.0 / coefficient2);
    }

    /**
     * Gets power ratio between received power and transmitted power.
     *
     * @param receivedPower    received power (expressed in mW).
     * @param transmittedPower transmitted power (expressed in mW).
     * @return power ratio.
     */
    public static double getRatio(
            final double receivedPower, final double transmittedPower) {
        return receivedPower / transmittedPower;
    }

    /**
     * Gets received power.
     *
     * @param ratio            ratio between received power and transmitted power.
     * @param transmittedPower transmitted power (expressed in mW).
     * @return received power (expressed in mW).
     */
    public static double getReceivedPower(
            final double ratio, final double transmittedPower) {
        //ratio = Pr / Pt
        return ratio * transmittedPower;
    }

    /**
     * Gets received power using AltBeacon's coefficients.
     *
     * @param coefficient1     1st coefficient.
     * @param coefficient2     2nd coefficient.
     * @param coefficient3     3rd coefficient.
     * @param distance         distance expressed in meters.
     * @param transmittedPower transmitted power expressed in mW.
     * @return received power expressed in mW.
     */
    public static double getReceivedPower(
            final double coefficient1, final double coefficient2,
            final double coefficient3, final double distance,
            final double transmittedPower) {
        return getReceivedPower(getRatio(coefficient1, coefficient2, coefficient3,
                distance), transmittedPower);
    }

    /**
     * Gets transmitted power.
     *
     * @param ratio         ratio between received power and transmitted power.
     * @param receivedPower received power expressed in mW.
     * @return transmitted power expressed in mW.
     */
    public static double getTransmittedPower(
            final double ratio, final double receivedPower) {
        //ratio = Pr / Pt
        return receivedPower / ratio;
    }

    /**
     * Gets transmitted power using AltBeacon's coefficients.
     *
     * @param coefficient1  1st coefficient.
     * @param coefficient2  2nd coefficient.
     * @param coefficient3  3rd coefficient.
     * @param distance      distance expressed in meters.
     * @param receivedPower received power expressed in mW.
     * @return transmitted power expressed in mW.
     */
    public static double getTransmittedPower(
            final double coefficient1, final double coefficient2,
            final double coefficient3, final double distance,
            final double receivedPower) {
        return getTransmittedPower(getRatio(coefficient1, coefficient2,
                coefficient3, distance), receivedPower);
    }

    /**
     * Gets AltBeacon's 1st coefficient.
     *
     * @param k                k value.
     * @param pathLossExponent path loss exponent.
     * @return AltBeacon's 1st coefficient.
     */
    public static double getCoefficient1(
            final double k, final double pathLossExponent) {
        //coefficient1 = k^(1/n)
        return Math.pow(k, 1.0 / pathLossExponent);
    }

    /**
     * Gets AltBeacon's 1st coefficient.
     *
     * @param frequency        frequency expressed in Hz.
     * @param pathLossExponent path loss exponent.
     * @return AltBeacon's 1st coefficient.
     */
    public static double getCoefficient1WithFrequency(
            final double frequency, final double pathLossExponent) {
        return getCoefficient1(getK(frequency), pathLossExponent);
    }

    /**
     * Gets AltBeacon's 2nd coefficient.
     *
     * @param pathLossExponent path loss exponent.
     * @return AltBeacon's 2nd coefficient.
     */
    public static double getCoefficient2(final double pathLossExponent) {
        //coefficient2 = -1/n
        return -1.0 / pathLossExponent;
    }
}
