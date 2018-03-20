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

public class Utils {

    /**
     * Prevents instantiation
     */
    private Utils() { }

    /**
     * Converts from dBm's to linear power value expressed in mW.
     * @param dBm value to be converted expressed in dBm's.
     * @return converted value expressed in mW.
     */
    public static double dBmToPower(double dBm) {
        return Math.pow(10.0, dBm / 10.0);
    }

    /**
     * Converts from mW to logarithmic power value expressed in dBm's.
     * @param mW value to be converted expressed in mW's.
     * @return converted value expressed in dBm's.
     */
    public static double powerTodBm(double mW) {
        return 10.0 * Math.log10(mW);
    }
}
