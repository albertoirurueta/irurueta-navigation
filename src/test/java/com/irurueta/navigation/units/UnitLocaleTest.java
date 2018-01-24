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
package com.irurueta.navigation.units;

import org.junit.*;

import java.util.Locale;

import static org.junit.Assert.*;

public class UnitLocaleTest {

    public UnitLocaleTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        //noinspection all
        assertNotNull(new UnitLocale());
    }

    @Test
    public void testGetDefault() {
        Locale defaultLocale = Locale.getDefault();
        boolean isImperial = false;
        if("US".equals(defaultLocale.getCountry()) ||
                "LR".equals(defaultLocale.getCountry()) ||
                "MM".equals(defaultLocale.getCountry())){
            isImperial = true;
        }

        assertEquals(UnitLocale.getDefault() == UnitSystem.IMPERIAL,
                isImperial);
    }

    @Test
    public void testGetFrom() {
        Locale l = new Locale("en", UnitLocale.USA);
        assertEquals(UnitLocale.getFrom(l), UnitSystem.IMPERIAL);

        l = new Locale("en", UnitLocale.LIBERIA);
        assertEquals(UnitLocale.getFrom(l), UnitSystem.IMPERIAL);

        l = new Locale("en", UnitLocale.BURMA);
        assertEquals(UnitLocale.getFrom(l), UnitSystem.IMPERIAL);

        l = new Locale("en", "GB");
        assertEquals(UnitLocale.getFrom(l), UnitSystem.METRIC);

        l = new Locale("es", "ES");
        assertEquals(UnitLocale.getFrom(l), UnitSystem.METRIC);
    }
}
