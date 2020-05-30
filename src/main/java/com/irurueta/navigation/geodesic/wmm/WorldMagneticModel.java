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
package com.irurueta.navigation.geodesic.wmm;

/**
 * Contains data defining the World Magnetic Model (WWM).
 *
 * The WWM defines the Earth's magnetic field in geodetic coordinates
 * (latitude, longitude and height) respect the WGS84 spheroid from
 * the coefficients of the current official Department of Defense (DOD)
 * spherical harmonic world magnetic model (WMM-2010).
 * The WWM series of models is updated every 5 years on January 1st of
 * those years, which are divisible by 5 (i.e. 1980, 1985, 1990, etc.)
 * by the Naval Oceanographic Office in cooperation with the
 * British Geological Survey (BGS).
 *
 * The model is based on geomagnetic survey measurements from aircraft,
 * satellite and geomagnetic observatories.
 */
public class WorldMagneticModel {

    /**
     * The maximum number of degrees of the spherical harmonic model.
     */
    public static final int MAX_ORDER = 12;

    /**
     * The lifespan of a WMM is 5 years.
     */
    public static final double LIFESPAN = 5.0;

    /**
     * Number of coefficients.
     */
    static final int N = 13;

    /**
     * Squared number of coefficients.
     */
    private static final int N2 = N * N;

    /**
     * The Gauss coefficients of main geomagneic model (nt)
     */
    final double[][] c = new double[N][N];

    /**
     * The Gauss coefficients of secular geomagnetic model (nt/yr).
     */
    final double[][] cd = new double[N][N];

    /**
     * The date in years, for the start of the valid time of the fit coefficients
     */
    double epoch;

    /**
     * The Schmidt normalization factors.
     */
    final double[] snorm = new double[N2];

    final double[][] k = new double[N][N];

    final double[] fn = new double[N];
    final double[] fm = new double[N];
}
