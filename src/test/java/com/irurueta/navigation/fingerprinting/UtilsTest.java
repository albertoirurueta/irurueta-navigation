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
package com.irurueta.navigation.fingerprinting;

import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class UtilsTest {

    private static final double ABSOLUTE_ERROR = 1e-6;

    public UtilsTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testdBmToPowerAndPowerTodBm() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double value = randomizer.nextDouble();

        assertEquals(Utils.powerTodBm(Utils.dBmToPower(value)), value,
                ABSOLUTE_ERROR);
        assertEquals(Utils.dBmToPower(Utils.powerTodBm(value)), value,
                ABSOLUTE_ERROR);
    }
}
