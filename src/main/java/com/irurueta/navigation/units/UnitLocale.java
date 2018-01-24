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

import java.util.Locale;

/**
 * Helper class to determine the appropriate unit system for a given locale.
 * Because detected unit system might not be 100% accurate, users should always
 * have the option to specify their own unit system if the applications requires
 * it.
 */
public class UnitLocale {

    /**
     * USA ISO country code.
     */
    public static final String USA = "US";

    /**
     * Liberia ISO country code.
     */
    public static final String LIBERIA = "LR";

    /**
     * Burma ISO country code.
     */
    public static final String BURMA = "MM";

    /**
     * Constructor.
     * Prevent instantiation of utility class.
     */
    UnitLocale() { }

    /**
     * Returns unit system for default locale.
     * @return unit system.
     */
    public static UnitSystem getDefault() {
        return getFrom(Locale.getDefault());
    }

    /**
     * Returns unit system for provided locale.
     * @param locale a locale indicating at least a country, if no country is
     * indicated then metric system will be assumed.
     * @return unis system.
     */
    public static UnitSystem getFrom(Locale locale) {
        String countryCode = locale.getCountry();
        if (USA.equals(countryCode)) {
            //USA
            return UnitSystem.IMPERIAL;
        }
        if (LIBERIA.equals(countryCode)) {
            //Liberia
            return UnitSystem.IMPERIAL;
        }
        if (BURMA.equals(countryCode)) {
            //Burma
            return UnitSystem.IMPERIAL;
        }
        return UnitSystem.METRIC;
    }
}
