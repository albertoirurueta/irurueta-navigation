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
package com.irurueta.navigation.geodesic;

/**
 * Bit masks for what geodesic calculations to do.
 * These masks do double duty. They specify (via the <i>outmask</i> parameter) which
 * results to return in the {@link GeodesicData} object returned by the general routines
 * {@link Geodesic#direct(double, double, double, double, int)} and
 * {@link Geodesic#inverse(double, double, double, double, int)} routines.
 * They also signify (via the <i>caps</i> parameter) to the
 * {@link GeodesicLine#geodesicLine(Geodesic, double, double, double, int)} constructor
 * and to {@link Geodesic#line(double, double, double, int)} what capabilities
 * should be included in the {@link GeodesicLine} object.
 */
public class GeodesicMask {
    protected static final int CAP_NONE = 0;

    protected static final int CAP_C1 = 1 << 0;

    protected static final int CAP_C1P = 1 << 1;

    protected static final int CAP_C2 = 1 << 2;

    protected static final int CAP_C3 = 1 << 3;

    protected static final int CAP_C4 = 1 << 4;

    protected static final int CAP_ALL = 0x1F;

    protected static final int CAP_MASK = CAP_ALL;

    protected static final int OUT_ALL = 0x7F80;

    //include LONG_UNROLL
    protected static final int OUT_MASK = 0xFF80;

    /**
     * No capabilities, no output.
     */
    public static final int NONE = 0;

    /**
     * Calculate latitude <i>lat2</i>. (It's not necessary to include this as a capability to
     * {@link GeodesicLine} because this is included by default.)
     */
    public static final int LATITUDE = 1 << 7 | CAP_NONE;

    /**
     * Calculate longitude <i>lon2</i>.
     */
    public static final int LONGITUDE = 1 << 8 | CAP_C3;

    /**
     * Calculate azimuths <i>azi1</i> and <i>azi2</i>. (It's not necessary to include this
     * as a capability to {@link GeodesicLine} because this is included by default.)
     */
    public static final int AZIMUTH = 1 << 9 | CAP_NONE;

    /**
     * Calculate distance <i>s12</i>.
     */
    public static final int DISTANCE = 1 << 10 | CAP_C1;

    /**
     * All of the above, the "standard" output and capabilities.
     */
    public static final int STANDARD = LATITUDE | LONGITUDE | AZIMUTH | DISTANCE;

    /**
     * Allow distance <i>s12</i> to be used as <i>input</i> in the direct geodesic problem.
     */
    public static final int DISTANCE_IN = 1 << 11 | CAP_C1 | CAP_C1P;

    /**
     * Calculate reduced length <i>m12</i>.
     */
    public static final int REDUCED_LENGTH = 1 << 12 | CAP_C1 | CAP_C2;

    /**
     * Calculate geodesic scales <i>M12</i> and <i>M21</i>.
     */
    public static final int GEODESIC_SCALE = 1 << 13 | CAP_C1 | CAP_C2;

    /**
     * Calculate area <i>S12</i>.
     */
    public static final int AREA = 1 << 14 | CAP_C4;

    /**
     * All capabilities, calculate everything. (LONG_UNROLL is not included in this mask.)
     */
    public static final int ALL = OUT_ALL | CAP_ALL;

    /**
     * Unroll <i>lon2</i>.
     */
    public static final int LONG_UNROLL = 1 << 15;
}
