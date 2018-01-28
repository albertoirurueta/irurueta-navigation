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

/**
 * Enumerator containing recognized typical time units.
 */
public enum TimeUnit {
    /**
     * Nanosecond time unit.
     */
    NANOSECOND,

    /**
     * Microsecond time unit.
     */
    MICROSECOND,

    /**
     * Millisecond time unit.
     */
    MILLISECOND,

    /**
     * Second time unit.
     */
    SECOND,

    /**
     * Minute time unit.
     */
    MINUTE,

    /**
     * Hour time unit.
     */
    HOUR,

    /**
     * Day time unit.
     */
    DAY,

    /**
     * Week time unit.
     */
    WEEK,

    /**
     * Month time unit (considered as 30 days).
     */
    MONTH,

    /**
     * Year time unit (considered as 365 days).
     */
    YEAR,

    /**
     * Century time unit (considered as 100 years).
     */
    CENTURY,
}
