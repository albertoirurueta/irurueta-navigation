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
 * The results of gnomonic projection.
 *
 * This is used to return the results for a gnomonic projection of a point
 * (<i>lat</i>, <i>lon</i>) given a center point of projection (<i>lat0</i>,
 * <i>lon0</i>). The returned GnomonicData objects always include the
 * parameters provided to
 * {@link Gnomonic@forward} and {@link Gnomonic#reverse}
 * and it always includes the fields <i>x</i>, <i>y</i>, <i>azi</i>, and
 * <i>rk</i>.
 */
public class GnomonicData {

    /**
     * Latitude of center point of projection (degrees).
     */
    public double lat0;

    /**
     * Longitude of center point of projection (degrees).
     */
    public double lon0;

    /**
     * Latitude of point (degrees).
     */
    public double lat;

    /**
     * Longitude of point (degrees).
     */
    public double lon;

    /**
     * Easting of point (meters).
     */
    public double x;

    /**
     * Northing of point (meters).
     */
    public double y;

    /**
     * Aximuth of geodesic at point (degrees).
     */
    public double azi;

    /**
     * Reciprocal of azimuthal scale at point.
     */
    public double rk;

    /**
     * Initialize all the fields to Double.NaN.
     */
    public GnomonicData() {
        lat0 = lon0 = lat = lon = x = y = azi = rk = Double.NaN;
    }

    /**
     * Constructor initializing all the fields for gnomonic projection of a
     * point (<i>lat</i>, <i>lon</i>) given a center point of projection
     * (<i>lat0</i>, <i>lon0</i>).
     * @param lat0 latitude of center point of projection (degrees).
     * @param lon0 longitude of center point of projection (degrees).
     * @param lat latitude of point (degrees).
     * @param lon longitude of point (degrees).
     * @param x easting of point (meters).
     * @param y northing of point (meters).
     * @param azi azimuth of geodesic at point (degrees).
     * @param rk reciprocal of azimuthal scale at point.
     */
    public GnomonicData(double lat0, double lon0, double lat, double lon,
                        double x, double y, double azi, double rk) {
        this.lat0 = lat0;
        this.lon0 = lon0;
        this.lat = lat;
        this.lon = lon;
        this.x = x;
        this.y = y;
        this.azi = azi;
        this.rk = rk;
    }
}
