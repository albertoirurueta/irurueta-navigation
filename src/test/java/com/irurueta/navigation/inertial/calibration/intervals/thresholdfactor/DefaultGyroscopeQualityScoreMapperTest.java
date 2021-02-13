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

import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;

public class DefaultGyroscopeQualityScoreMapperTest {

    private static final double MAX_SPECIFIC_FORCE = 9.81;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final int NUM_ITEMS = 100;

    @Test
    public void testMap() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double avg = 0.0;
        final List<StandardDeviationTimedBodyKinematics> items = new ArrayList<>();
        for (int i = 0; i < NUM_ITEMS; i++) {
            final double specificForceStandardDeviation =
                    randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
            final double angularRateStandardDeviation =
                    randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
            final StandardDeviationTimedBodyKinematics item =
                    new StandardDeviationTimedBodyKinematics(TIME_INTERVAL * i,
                            specificForceStandardDeviation, angularRateStandardDeviation);
            items.add(item);

            avg += (specificForceStandardDeviation + angularRateStandardDeviation) / NUM_ITEMS;
        }

        final double expected = 1.0 / (1.0 + avg);

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>(items);

        final DefaultGyroscopeQualityScoreMapper mapper = new DefaultGyroscopeQualityScoreMapper();
        assertEquals(expected, mapper.map(sequence), 0.0);
    }
}
