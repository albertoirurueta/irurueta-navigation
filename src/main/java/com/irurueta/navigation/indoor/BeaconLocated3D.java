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
 * Data related to a beacon whose 3D location is known.
 */
public class BeaconLocated3D extends BeaconLocated<Point3D> {

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI
     *                   (expressed in dBm's).
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            Point3D position) {
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
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            String bluetoothAddress, int beaconTypeCode,
            int manufacturer, int serviceUuid, String bluetoothName,
            Point3D position) {
        super(identifiers, transmittedPower, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName, position);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI
     *                   (expressed in dBm's).
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            Point3D position, Matrix positionCovariance) {
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
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            String bluetoothAddress, int beaconTypeCode,
            int manufacturer, int serviceUuid, String bluetoothName,
            Point3D position, Matrix positionCovariance) {
        super(identifiers, transmittedPower, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName, position,
                positionCovariance);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * frequency is negative.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            double frequency, Point3D position) {
        super(identifiers, transmittedPower, frequency, position);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac address.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param position position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     * frequency is negative.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            double frequency, String bluetoothAddress, int beaconTypeCode,
            int manufacturer, int serviceUuid, String bluetoothName, Point3D position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName, position);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI
     *                   (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     * frequency is negative.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            double frequency, Point3D position, Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, position, positionCovariance);
    }

    /**
     * Constructor.
     * @param identifiers list of the multi-part idntifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac address.
     * @param beaconTypeCode the two byte value indicating the type of beacon.
     * @param manufacturer a two byte code indicating the beacon manufacturer.
     * @param serviceUuid a 32 bit service uuid for the beacon.
     * @param bluetoothName the bluetooth device name.
     * @param position position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconLocated3D(List<BeaconIdentifier> identifiers, double transmittedPower,
            double frequency, String bluetoothAddress, int beaconTypeCode,
            int manufacturer, int serviceUuid, String bluetoothName, Point3D position,
            Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress, beaconTypeCode,
                manufacturer, serviceUuid, bluetoothName, position, positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected BeaconLocated3D() {
        super();
    }
}
