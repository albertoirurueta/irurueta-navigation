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

import java.util.List;

/**
 * Data related to a beacon whose transmitted power and standard deviation of
 * such transmitted power is known.
 */
@SuppressWarnings("WeakerAccess")
public class BeaconWithPower extends Beacon implements RadioSourceWithPower {

    /**
     * Default exponent typically used on free space for path loss propagation in
     * terms of distance. This value is used for free space environments.
     */
    public static final double DEFAULT_PATH_LOSS_EXPONENT = 2.0;

    /**
     * Standard deviation of transmitted power value or null if unknown.
     */
    private Double mTransmittedPowerStandardDeviation;

    /**
     * Exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     */
    private double mPathLossExponent = DEFAULT_PATH_LOSS_EXPONENT;

    /**
     * Standard deviation of path loss exponent or null if unknown.
     */
    private Double mPathLossExponentStandardDeviation;

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower) throws IllegalArgumentException {
        super(identifiers, transmittedPower);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null or standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency,
            String bluetoothAddress, int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double pathLossExponent) throws IllegalArgumentException {
        this(identifiers, transmittedPower);
        mPathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);
        mPathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @throws IllegalArgumentException if identifiers is null or standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent) throws IllegalArgumentException {
        this(identifiers, transmittedPower, pathLossExponent);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null or standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent,
            Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);
        mPathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null or standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation);
        mPathLossExponent = pathLossExponent;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @throws IllegalArgumentException if identifiers is null or standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency,
            String bluetoothAddress, int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);
        mPathLossExponent = pathLossExponent;

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @throws IllegalArgumentException if identifiers is null or any standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, pathLossExponent);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        if (pathLossExponentStandardDeviation != null && pathLossExponentStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
        mPathLossExponentStandardDeviation = pathLossExponentStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @throws IllegalArgumentException if identifiers is null or any standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent);

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        if (pathLossExponentStandardDeviation != null && pathLossExponentStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
        mPathLossExponentStandardDeviation = pathLossExponentStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @throws IllegalArgumentException if identifiers is null or any standard deviation is negative.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation);
        mPathLossExponent = pathLossExponent;

        if (pathLossExponentStandardDeviation != null && pathLossExponentStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mPathLossExponentStandardDeviation = pathLossExponentStandardDeviation;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @throws IllegalArgumentException if identifiers is null.
     */
    public BeaconWithPower(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency,
            String bluetoothAddress, int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);
        mPathLossExponent = pathLossExponent;

        if (transmittedPowerStandardDeviation != null && transmittedPowerStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }
        if (pathLossExponentStandardDeviation != null && pathLossExponentStandardDeviation < 0.0) {
            throw new IllegalArgumentException();
        }

        mTransmittedPowerStandardDeviation = transmittedPowerStandardDeviation;
        mPathLossExponentStandardDeviation = pathLossExponentStandardDeviation;
    }

    /**
     * Empty constructor.
     */
    protected BeaconWithPower() {
        super();
    }

    /**
     * Gets standard deviation of transmitted power value or null if unknown.
     * @return standard deviation of transmitted power value or null if unknown.
     */
    @Override
    public Double getTransmittedPowerStandardDeviation() {
        return mTransmittedPowerStandardDeviation;
    }

    /**
     * Gets exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * If path loss exponent estimation is not enabled, this value will always be equal to
     * {@link #DEFAULT_PATH_LOSS_EXPONENT}
     * @return path loss exponent.
     */
    @Override
    public double getPathLossExponent() {
        return mPathLossExponent;
    }

    /**
     * Gets standard deviation of path loss exponent or null if unknown.
     * @return standard deviation of path loss exponent or null if unknown.
     */
    @Override
    public Double getPathLossExponentStandardDeviation() {
        return mPathLossExponentStandardDeviation;
    }
}
