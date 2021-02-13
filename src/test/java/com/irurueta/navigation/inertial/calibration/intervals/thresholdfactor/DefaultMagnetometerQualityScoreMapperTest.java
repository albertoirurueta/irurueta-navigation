/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;

import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class DefaultMagnetometerQualityScoreMapperTest {

    private static final double MAX_MAGNETIC_FLUX_STD = 1e-9;

    @Test
    public void testMap() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double standardDeviation = randomizer.nextDouble(
                0.0, MAX_MAGNETIC_FLUX_STD);
        final StandardDeviationBodyMagneticFluxDensity stdB =
                new StandardDeviationBodyMagneticFluxDensity(standardDeviation);

        final DefaultMagnetometerQualityScoreMapper mapper = new DefaultMagnetometerQualityScoreMapper();

        assertEquals(1.0 / (1.0 + standardDeviation), mapper.map(stdB), 0.0);
    }
}
