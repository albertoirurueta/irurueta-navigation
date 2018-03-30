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

import com.irurueta.geometry.Point;

/**
 * Interface defining any radio source (e.g. WiFi access point or
 * Bluetooth beacon) whose transmitted power and location are known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public interface RadioSourceWithPowerAndLocated<P extends Point>
        extends RadioSourceWithPower, RadioSourceLocated<P> { }
