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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;

import java.util.List;

/**
 * Data related to a beacon whose transmitted power, standard deviation of such
 * transmitted power and its location are known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public class BeaconWithPowerAndLocated<P extends Point> extends BeaconWithPower
        implements RadioSourceWithPowerAndLocated<P> {

    /**
     * Position where beacon is located.
     */
    private P mPosition;

    /**
     * Covariance of inhomogeneous coordinates of current position (if available).
     */
    private Matrix mPositionCovariance;

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, P position) {
        super(identifiers, transmittedPower);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, P position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            P position) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, P position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, P position, Matrix positionCovariance) {
        this(identifiers, transmittedPower, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, P position,
            Matrix positionCovariance) {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            P position, Matrix positionCovariance) {
        this(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, P position,
            Matrix positionCovariance) {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, P position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, Double transmittedPowerStandardDeviation,
            P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, P position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency,
            Double transmittedPowerStandardDeviation,
            P position, Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation,
                position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double pathLossExponent, P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent, Double transmittedPowerStandardDeviation,
            P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double pathLossExponent,
            P position, Matrix positionCovariance)
            throws IllegalArgumentException {
        this(identifiers, transmittedPower, pathLossExponent, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent, P position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation, P position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer, int serviceUuid,
            String bluetoothName, double pathLossExponent, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            P position, Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent, Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation,
            P position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, P position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
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
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, P position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation,
            P position, Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, pathLossExponentStandardDeviation,
                position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
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
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, P position,
            Matrix positionCovariance) throws IllegalArgumentException {
        this(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);

        if(positionCovariance != null &&
                (positionCovariance.getRows() != position.getDimensions() ||
                        positionCovariance.getColumns() != position.getDimensions())) {
            throw new IllegalArgumentException();
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected BeaconWithPowerAndLocated() {
        super();
    }

    /**
     * Gets position where beacon is located.
     * @return position where beacon is located.
     */
    @Override
    public P getPosition() {
        return mPosition;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     * @return covariance of position or null.
     */
    @Override
    public Matrix getPositionCovariance() {
        return mPositionCovariance;
    }
}
