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
import com.irurueta.geometry.Point3D;

import java.util.List;

/**
 * Data related to a beacon whose transmitted power, standard deviation of such
 * transmitted power and its 3D location are known.
 */
@SuppressWarnings("WeakerAccess")
public class BeaconWithPowerAndLocated3D extends BeaconWithPowerAndLocated<Point3D> {

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                position);
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
     * @throws IllegalArgumentException if either identifiers or position are null or
     * standard deviation is negative.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation,
                position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Point3D position, Matrix positionCovariance)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position, positionCovariance);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * covariance has invalid size.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            Point3D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                position, positionCovariance);
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
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position,
                positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, Double transmittedPowerStandardDeviation,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation,
                position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency,
            Double transmittedPowerStandardDeviation,
            Point3D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation,
                position, positionCovariance);
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
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            Double transmittedPowerStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position,
                positionCovariance);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double pathLossExponent, Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, pathLossExponent, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent, Double transmittedPowerStandardDeviation,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double pathLossExponent,
            Point3D position, Matrix positionCovariance)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, pathLossExponent, position,
                positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                position, positionCovariance);
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
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation, position,
                positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer, int serviceUuid,
            String bluetoothName, double pathLossExponent, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Point3D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, position, positionCovariance);
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
     * @throws IllegalArgumentException if either identifiers or position are null,
     * covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName,
            double pathLossExponent, Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation,
            Point3D position) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, Double transmittedPowerStandardDeviation,
            double pathLossExponent, Double pathLossExponentStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, Point3D position)
            throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation,
            Point3D position, Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, pathLossExponentStandardDeviation,
                position, positionCovariance);
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
    public BeaconWithPowerAndLocated3D(List<BeaconIdentifier> identifiers,
            double transmittedPower, double frequency, String bluetoothAddress,
            int beaconTypeCode, int manufacturer,
            int serviceUuid, String bluetoothName, double pathLossExponent,
            Double transmittedPowerStandardDeviation,
            Double pathLossExponentStandardDeviation, Point3D position,
            Matrix positionCovariance) throws IllegalArgumentException {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position, positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected BeaconWithPowerAndLocated3D() {
        super();
    }
}
