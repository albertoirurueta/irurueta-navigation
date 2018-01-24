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
 * The results of geodesic calculations.
 * This is used to return the results for a geodesic between point 1 (<i>lat1</i>, <i>lon1</i>)
 * and point 2 (<i>lat2</i>, <i>lon2</i>).
 * Fields that have not been set will be filled with Double.NaN.
 * The returned GeodesicData objects always include the parameters provided to
 * {@link Geodesic#direct(double, double, double, double)} and
 * {@link Geodesic#inverse(double, double, double, double)} and it always includes the field
 * <i>a12</i>.
 */
public class GeodesicData {

    /**
     * Latitude of point 1 (degrees).
     */
    public double lat1;

    /**
     * Longitude of point 1 (degrees).
     */
    public double lon1;

    /**
     * Azimuth at point 1 (degrees).
     */
    public double azi1;

    /**
     * Latitude of point 2 (degrees).
     */
    public double lat2;

    /**
     * Longitude of point 2 (degrees).
     */
    public double lon2;

    /**
     * Azimuth at point 2 (degrees).
     */
    public double azi2;

    /**
     * Distance between point 1 and point 2 (meters).
     */
    public double s12;

    /**
     * Arc length on the auxiliary sphere between point 1 and point 2 (degrees).
     */
    public double a12;

    /**
     * Reduced length of geodesic (meters).
     */
    public double m12;

    /**
     * Geodesic scale of point 2 relative to point 1 (dimensionless).
     */
    public double M12;

    /**
     * Geodesic scale of point 1 relative to point 2 (dimensionless).
     */
    public double M21;

    /**
     * Area under the geodesic (meters<sup>2</sup>).
     */
    public double S12;

    /**
     * Constructor.
     * Initialize all the fields to Double.NaN.
     */
    public GeodesicData() {
        lat1 = lon1 = azi1 = lat2 = lon2 = azi2 = s12 = a12 = m12 = M12 = M21 = S12 = Double.NaN;
    }
}
