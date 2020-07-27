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
 * <p>
 * This is used to return the results for a gnomonic projection of a point
 * (<i>lat</i>, <i>lon</i>) given a center point of projection (<i>lat0</i>,
 * <i>lon0</i>). The returned GnomonicData objects always include the
 * parameters provided to
 * {@link Gnomonic#forward} and {@link Gnomonic#reverse}
 * and it always includes the fields <i>x</i>, <i>y</i>, <i>azi</i>, and
 * <i>rk</i>.
 */
@SuppressWarnings("WeakerAccess")
public class GnomonicData {

    /**
     * Latitude of center point of projection (degrees).
     */
    private double lat0;

    /**
     * Longitude of center point of projection (degrees).
     */
    private double lon0;

    /**
     * Latitude of point (degrees).
     */
    private double lat;

    /**
     * Longitude of point (degrees).
     */
    private double lon;

    /**
     * Easting of point (meters).
     */
    private double x;

    /**
     * Northing of point (meters).
     */
    private double y;

    /**
     * Azimuth of geodesic at point (degrees).
     */
    private double azi;

    /**
     * Reciprocal of azimuthal scale at point.
     */
    private double rk;

    /**
     * Initialize all the fields to Double.NaN.
     */
    @SuppressWarnings("WeakerAccess")
    public GnomonicData() {
        lat0 = lon0 = lat = lon = x = y = azi = rk = Double.NaN;
    }

    /**
     * Constructor initializing all the fields for gnomonic projection of a
     * point (<i>lat</i>, <i>lon</i>) given a center point of projection
     * (<i>lat0</i>, <i>lon0</i>).
     *
     * @param lat0 latitude of center point of projection (degrees).
     * @param lon0 longitude of center point of projection (degrees).
     * @param lat  latitude of point (degrees).
     * @param lon  longitude of point (degrees).
     * @param x    easting of point (meters).
     * @param y    northing of point (meters).
     * @param azi  azimuth of geodesic at point (degrees).
     * @param rk   reciprocal of azimuthal scale at point.
     */
    @SuppressWarnings("WeakerAccess")
    public GnomonicData(
            final double lat0, final double lon0, final double lat, final double lon,
            final double x, final double y, final double azi, final double rk) {
        this.lat0 = lat0;
        this.lon0 = lon0;
        this.lat = lat;
        this.lon = lon;
        this.x = x;
        this.y = y;
        this.azi = azi;
        this.rk = rk;
    }

    /**
     * Gets latitude of center point of projection (degrees).
     *
     * @return latitude of center point of projection.
     */
    public double getLat0() {
        return lat0;
    }

    /**
     * Sets latitude of center point of projection (degrees).
     *
     * @param lat0 latitude of center point of projection.
     */
    public void setLat0(final double lat0) {
        this.lat0 = lat0;
    }

    /**
     * Gets longitude of center point of projection (degrees).
     *
     * @return longitude of center point of projection.
     */
    public double getLon0() {
        return lon0;
    }

    /**
     * Sets longitude of center point of projection (degrees).
     *
     * @param lon0 longitude of center point of projection.
     */
    public void setLon0(final double lon0) {
        this.lon0 = lon0;
    }

    /**
     * Gets latitude of point (degrees).
     *
     * @return latitude of point.
     */
    public double getLat() {
        return lat;
    }

    /**
     * Sets latitude of point (degrees).
     *
     * @param lat latitude of point.
     */
    public void setLat(final double lat) {
        this.lat = lat;
    }

    /**
     * Gets longitude of point (degrees).
     *
     * @return longitude of point.
     */
    public double getLon() {
        return lon;
    }

    /**
     * Sets longitude of point (degrees).
     *
     * @param lon longitude of point.
     */
    public void setLon(final double lon) {
        this.lon = lon;
    }

    /**
     * Gets easting of point (meters).
     *
     * @return easting of point.
     */
    public double getX() {
        return x;
    }

    /**
     * Sets easting of point (meters).
     *
     * @param x easting of point.
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Gets northing of point (meters).
     *
     * @return northing of point.
     */
    public double getY() {
        return y;
    }

    /**
     * Sets northing of point (meters).
     *
     * @param y northing of point.
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Gets azimuth of geodesic at point (degrees).
     *
     * @return azimuth of geodesic at point.
     */
    public double getAzi() {
        return azi;
    }

    /**
     * Sets azimuth of geodesic at point (degrees).
     *
     * @param azi azimuth of geodesic at point.
     */
    public void setAzi(final double azi) {
        this.azi = azi;
    }

    /**
     * Gets reciprocal of azimuthal scale at point.
     *
     * @return reciprocal of azimuthal scale at point.
     */
    public double getRk() {
        return rk;
    }

    /**
     * Sets reciprocal of azimuthal scale at point.
     *
     * @param rk reciprocal of azimuthal scale at point.
     */
    public void setRk(final double rk) {
        this.rk = rk;
    }
}
