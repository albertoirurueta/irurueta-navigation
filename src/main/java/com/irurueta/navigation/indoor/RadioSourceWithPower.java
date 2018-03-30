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
 * Interface defining any radio source (e.g. WiFi access point or
 * Bluetooth beacon) whose transmitted power is known.
 */
public interface RadioSourceWithPower extends RadioSource {
    /**
     * Gets transmitted power expressed in dBm's.
     * @return transmitted power expressed in dBm's.
     */
    double getTransmittedPower();

    /**
     * Gets standard deviation of transmitted power value or null if unknown.
     * @return standard deviation of transmitted power value or null if unknown.
     */
    Double getTransmittedPowerStandardDeviation();

    /**
     * Gets exponent typically used on free space for path loss propagation in
     * terms of distance.
     * On different environments path loss exponent might have different values:
     * - Free space: 2.0
     * - Urban Area: 2.7 to 3.5
     * - Suburban Area: 3 to 5
     * - Indoor (line-of-sight): 1.6 to 1.8
     * @return path loss exponent.
     */
    double getPathLossExponent();

    /**
     * Gets standard deviation of path loss exponent or null if unknown.
     * @return standard deviation of path loss exponent or null if unknown.
     */
    Double getPathLossExponentStandardDeviation();
}
