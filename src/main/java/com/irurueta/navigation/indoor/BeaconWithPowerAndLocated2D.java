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
import com.irurueta.geometry.Point2D;

import java.util.List;

/**
 * Data related to a beacon whose transmitted power, standard deviation of such
 * transmitted power and its 2D location are known.
 */
public class BeaconWithPowerAndLocated2D extends BeaconWithPowerAndLocated<Point2D> {

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Point2D position) {
        super(identifiers, transmittedPower, position);
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
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Point2D position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers, position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation,
                position);
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, position, positionCovariance);
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
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position,
                positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency        frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode   the two byte value indicating the type of beacon.
     * @param manufacturer     a two byte code indicating the beacon manufacturer.
     * @param serviceUuid      a 32 bit service uuid for the beacon.
     * @param bluetoothName    the bluetooth device name.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation,
                position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency          frequency used by this Beacon.
     * @param bluetoothAddress   the bluetooth mac addres.
     * @param beaconTypeCode     the two byte value indicating the type of beacon.
     * @param manufacturer       a two byte code indicating the beacon manufacturer.
     * @param serviceUuid        a 32 bit service uuid for the beacon.
     * @param bluetoothName      the bluetooth device name.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, transmittedPowerStandardDeviation,
                position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, transmittedPowerStandardDeviation, position,
                positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double pathLossExponent,
            final Point2D position) {
        super(identifiers, transmittedPower, pathLossExponent, position);
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
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Point2D position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Double transmittedPowerStandardDeviation,
            final double pathLossExponent,
            final Point2D position) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation,
                pathLossExponent, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                position);
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param pathLossExponent   path loss exponent. By default this is 2.0.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double pathLossExponent,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, pathLossExponent, position,
                positionCovariance);
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
     * @param pathLossExponent   path loss exponent. By default this is 2.0.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Double transmittedPowerStandardDeviation,
            final double pathLossExponent,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation, position,
                positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers      list of the multi-part identifiers of the beacon.
     * @param transmittedPower calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency        frequency used by this Beacon.
     * @param bluetoothAddress the bluetooth mac addres.
     * @param beaconTypeCode   the two byte value indicating the type of beacon.
     * @param manufacturer     a two byte code indicating the beacon manufacturer.
     * @param serviceUuid      a 32 bit service uuid for the beacon.
     * @param bluetoothName    the bluetooth device name.
     * @param pathLossExponent path loss exponent. By default this is 2.0.
     * @param position         position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                position);
    }

    /**
     * Constructor.
     *
     * @param identifiers        list of the multi-part identifiers of the beacon.
     * @param transmittedPower   calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency          frequency used by this Beacon.
     * @param bluetoothAddress   the bluetooth mac addres.
     * @param beaconTypeCode     the two byte value indicating the type of beacon.
     * @param manufacturer       a two byte code indicating the beacon manufacturer.
     * @param serviceUuid        a 32 bit service uuid for the beacon.
     * @param bluetoothName      the bluetooth device name.
     * @param pathLossExponent   path loss exponent. By default this is 2.0.
     * @param position           position where beacon is located.
     * @param positionCovariance covariance of inhomogeneous coordinates of current
     *                           position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  covariance has invalid size.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation transmitted power standard deviation.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Double transmittedPowerStandardDeviation,
            final double pathLossExponent,
            final Double pathLossExponentStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Double pathLossExponentStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final Double transmittedPowerStandardDeviation,
            final double pathLossExponent,
            final Double pathLossExponentStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, transmittedPowerStandardDeviation, pathLossExponent,
                pathLossExponentStandardDeviation, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Double pathLossExponentStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Double pathLossExponentStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @throws IllegalArgumentException if either identifiers or position are null or
     *                                  any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Double pathLossExponentStandardDeviation,
            final Point2D position) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Double pathLossExponentStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, pathLossExponent,
                transmittedPowerStandardDeviation, pathLossExponentStandardDeviation,
                position, positionCovariance);
    }

    /**
     * Constructor.
     *
     * @param identifiers                       list of the multi-part identifiers of the beacon.
     * @param transmittedPower                  calibrated measured Tx power of the Beacon in RSSI (expressed in dBm's).
     * @param frequency                         frequency used by this Beacon.
     * @param bluetoothAddress                  the bluetooth mac addres.
     * @param beaconTypeCode                    the two byte value indicating the type of beacon.
     * @param manufacturer                      a two byte code indicating the beacon manufacturer.
     * @param serviceUuid                       a 32 bit service uuid for the beacon.
     * @param bluetoothName                     the bluetooth device name.
     * @param pathLossExponent                  path loss exponent. By default this is 2.0.
     * @param transmittedPowerStandardDeviation standard deviation of transmitted power value or null
     *                                          if unknown.
     * @param pathLossExponentStandardDeviation standard deviation of path loss exponent or null if
     *                                          unknown.
     * @param position                          position where beacon is located.
     * @param positionCovariance                covariance of inhomogeneous coordinates of current
     *                                          position (if available).
     * @throws IllegalArgumentException if either identifiers or position are null,
     *                                  covariance has invalid size or any standard deviation is negative.
     */
    public BeaconWithPowerAndLocated2D(
            final List<BeaconIdentifier> identifiers,
            final double transmittedPower,
            final double frequency,
            final String bluetoothAddress,
            final int beaconTypeCode,
            final int manufacturer,
            final int serviceUuid,
            final String bluetoothName,
            final double pathLossExponent,
            final Double transmittedPowerStandardDeviation,
            final Double pathLossExponentStandardDeviation,
            final Point2D position,
            final Matrix positionCovariance) {
        super(identifiers, transmittedPower, frequency, bluetoothAddress,
                beaconTypeCode, manufacturer, serviceUuid,
                bluetoothName, pathLossExponent, transmittedPowerStandardDeviation,
                pathLossExponentStandardDeviation, position, positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected BeaconWithPowerAndLocated2D() {
        super();
    }
}
