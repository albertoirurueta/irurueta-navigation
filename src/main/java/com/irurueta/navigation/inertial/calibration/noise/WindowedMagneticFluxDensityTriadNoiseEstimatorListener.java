/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;

/**
 * Listener for {@link WindowedMagneticFluxDensityTriadNoiseEstimator} to handle generated events.
 */
public interface WindowedMagneticFluxDensityTriadNoiseEstimatorListener extends
        WindowedTriadNoiseEstimatorListener<MagneticFluxDensityUnit, MagneticFluxDensity,
                        MagneticFluxDensityTriad, WindowedMagneticFluxDensityTriadNoiseEstimator> {
}
