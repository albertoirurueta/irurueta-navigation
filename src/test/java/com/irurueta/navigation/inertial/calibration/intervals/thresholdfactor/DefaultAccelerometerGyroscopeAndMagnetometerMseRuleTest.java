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

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class DefaultAccelerometerGyroscopeAndMagnetometerMseRuleTest {

    @Test
    public void testEvaluate() {
        final DefaultAccelerometerGyroscopeAndMagnetometerMseRule rule =
                new DefaultAccelerometerGyroscopeAndMagnetometerMseRule();

        final Random random = new Random();
        final double accelerometerMse = random.nextDouble();
        final double gyroscopeMse = random.nextDouble();
        final double magnetometerMse = random.nextDouble();

        assertEquals(accelerometerMse + gyroscopeMse + magnetometerMse,
                rule.evaluate(accelerometerMse, gyroscopeMse, magnetometerMse),
                0.0);
    }
}
