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
 * Data related to a beacon whose location is known.
 *
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public class BeaconLocated<P extends Point<?>> extends Beacon
        implements RadioSourceLocated<P> {

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
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI
     *                         (expressed in dBm's).
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final P position) {
        super(identifiers, transmittedPower);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode   the two byte value indicating the type of beacon.
     * @param manufacturer     a two byte code indicating the beacon manufacturer.
     * @param serviceUuid      a 32 bit service uuid for the beacon.
     * @param bluetoothName    the bluetooth device name.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final String bluetoothAddress,
                         final int beaconTypeCode,
                         final int manufacturer,
                         final int serviceUuid,
                         final String bluetoothName,
                         final P position) {
        super(identifiers, transmittedPower, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI
     *                           (expressed in dBm's).
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final P position,
                         final Matrix positionCovariance) {
        this(identifiers, transmittedPower, position);

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress   the bluetooth mac addres.
     * @param beaconTypeCode     the two byte value indicating the type of beacon.
     * @param manufacturer       a two byte code indicating the beacon manufacturer.
     * @param serviceUuid        a 32 bit service uuid for the beacon.
     * @param bluetoothName      the bluetooth device name.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final String bluetoothAddress,
                         final int beaconTypeCode,
                         final int manufacturer,
                         final int serviceUuid,
                         final String bluetoothName,
                         final P position,
                         final Matrix positionCovariance) {
        this(identifiers, transmittedPower, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName, position);

        if (positionCovariance != null) {
            final int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI
     *                         (expressed in dBm's).
     * @param frequency        frequency used by this Beacon.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  frequency is negative..
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final double frequency,
                         final P position) {
        super(identifiers, transmittedPower, frequency);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency        frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac address.
     * @param beaconTypeCode   the two byte value indicating the type of beacon.
     * @param manufacturer     a two byte code indicating the beacon manufacturer.
     * @param serviceUuid      a 32 bit service uuid for the beacon.
     * @param bluetoothName    the bluetooth device name.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  frequency is negative.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final double frequency,
                         final String bluetoothAddress,
                         final int beaconTypeCode,
                         final int manufacturer,
                         final int serviceUuid,
                         final String bluetoothName,
                         final P position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName);

        if (position == null) {
            throw new IllegalArgumentException();
        }

        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI
     *                           (expressed in dBm's).
     * @param frequency          frequency used by this Beacon.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  frequency is negative.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final double frequency,
                         final P position,
                         final Matrix positionCovariance) {
        this(identifiers, transmittedPower, frequency, position);

        if (positionCovariance != null) {
            final int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency          frequency used by this Beacon.
     * @param bluetoothAddress   the bluetooth mac address.
     * @param beaconTypeCode     the two byte value indicating the type of beacon.
     * @param manufacturer       a two byte code indicating the beacon manufacturer.
     * @param serviceUuid        a 32 bit service uuid for the beacon.
     * @param bluetoothName      the bluetooth device name.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  frequency is negative.
     */
    public BeaconLocated(final List<BeaconIdentifier> identifiers,
                         final double transmittedPower,
                         final double frequency,
                         final String bluetoothAddress,
                         final int beaconTypeCode,
                         final int manufacturer,
                         final int serviceUuid,
                         final String bluetoothName,
                         final P position,
                         final Matrix positionCovariance) {
        this(identifiers, transmittedPower, frequency, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName, position);

        if (positionCovariance != null) {
            int dims = position.getDimensions();
            if (positionCovariance.getRows() != dims ||
                    positionCovariance.getColumns() != dims) {
                throw new IllegalArgumentException();
            }
        }
        mPositionCovariance = positionCovariance;
    }

    /**
     * Empty constructor.
     */
    protected BeaconLocated() {
        super();
    }

    /**
     * Gets position where beacon is located.
     *
     * @return position where beacon is located.
     */
    public P getPosition() {
        return mPosition;
    }

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     *
     * @return covariance of position or null.
     */
    public Matrix getPositionCovariance() {
        return mPositionCovariance;
    }
}
