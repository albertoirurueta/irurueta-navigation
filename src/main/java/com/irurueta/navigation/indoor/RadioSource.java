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
 * Interface defining any radio source (e.g. WiFi access point or Bluetooth beacon).
 */
public interface RadioSource extends Serializable {

    /**
     * Gets frequency used by this radio source (expressed in Hz).
     * @return frequency used by this radio source (expressed in Hz).
     */
    double getFrequency();

    /**
     * Checks whether two radio sources are considered equal if they share the same identifiers.
     * @param obj radio source to be compared.
     * @return true if both radio sources are considered equal, false otherwise.
     */
    boolean equals(final Object obj);

    /**
     * Gets radio source type, which can be either a WiFi Access point or a bluetooth Beacon.
     * @return radio source type.
     */
    RadioSourceType getType();
}
