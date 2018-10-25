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

/**
 * Contains structures to estimate positions and radio sources, as well
 * as required measurements to make such estimations.
 * There are two kinds of radio sources:
 * - Beacons
 * - Wifi Access Points
 *
 * All radio sources can be located (either in 2D or 3D), can have power measurements
 * (RSSI and pathloss exponent), or can be both located and have power measurements.
 *
 * Radio sources and positions can be estimated by making signal readings.
 * There are two kinds of readings:
 * - RSSI
 * - Ranging
 *
 * RSSI readings contain measurements of received signal power for a given radio source.
 * Ranging readings contains distance measurements for a given radio source.
 *
 * Readings can also contains RSSI and ranging data at the same time, and can be
 * located (either in 2D or 3D).
 *
 * One reading can only be associated to a single radio source.
 *
 * Readings can be grouped into fingerprints.
 * A fingerprint is a set of readings belonging to different radio sources that where measured
 * at a given time on the same location.
 *
 * Fingerprints can be used to estimate either radio sources or positions (in 2D or 3D).
 */
package com.irurueta.navigation.indoor;