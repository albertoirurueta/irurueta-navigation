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
@SuppressWarnings("WeakerAccess")
public class GeodesicData {

    /**
     * Latitude of point 1 (degrees).
     */
    private double lat1;

    /**
     * Longitude of point 1 (degrees).
     */
    private double lon1;

    /**
     * Azimuth at point 1 (degrees).
     */
    private double azi1;

    /**
     * Latitude of point 2 (degrees).
     */
    private double lat2;

    /**
     * Longitude of point 2 (degrees).
     */
    private double lon2;

    /**
     * Azimuth at point 2 (degrees).
     */
    private double azi2;

    /**
     * Distance between point 1 and point 2 (meters).
     */
    private double s12;

    /**
     * Arc length on the auxiliary sphere between point 1 and point 2 (degrees).
     */
    private double a12;

    /**
     * Reduced length of geodesic (meters).
     */
    private double m12;

    /**
     * Geodesic scale of point 2 relative to point 1 (dimensionless).
     */
    public double scaleM12;

    /**
     * Geodesic scale of point 1 relative to point 2 (dimensionless).
     */
    public double scaleM21;

    /**
     * Area under the geodesic (meters<sup>2</sup>).
     */
    private double areaS12;

    /**
     * Constructor.
     * Initialize all the fields to Double.NaN.
     */
    public GeodesicData() {
        lat1 = lon1 = azi1 = lat2 = lon2 = azi2 = s12 = a12 = m12 =
                scaleM12 = scaleM21 = areaS12 = Double.NaN;
    }

    /**
     * Gets latitude of point 1 (degrees).
     * @return latitude of point 1.
     */
    public double getLat1() {
        return lat1;
    }

    /**
     * Sets latitude of point 1 (degrees).
     * @param lat1 latitude of point 1.
     */
    public void setLat1(double lat1) {
        this.lat1 = lat1;
    }

    /**
     * Gets longitude of point 1 (degrees).
     * @return longitude of point 1.
     */
    public double getLon1() {
        return lon1;
    }

    /**
     * Sets longitude of point 1 (degrees).
     * @param lon1 longitude of point 1.
     */
    public void setLon1(double lon1) {
        this.lon1 = lon1;
    }

    /**
     * Gets azimuth at point 1 (degrees).
     * @return azimuth at point 1.
     */
    public double getAzi1() {
        return azi1;
    }

    /**
     * Sets azimuth at point 1 (degrees).
     * @param azi1 azimuth at point 1.
     */
    public void setAzi1(double azi1) {
        this.azi1 = azi1;
    }

    /**
     * Gets latitude of point 2 (degrees).
     * @return latitude of point 2.
     */
    public double getLat2() {
        return lat2;
    }

    /**
     * Sets latitude of point 2 (degrees).
     * @param lat2 latitude of point 2.
     */
    public void setLat2(double lat2) {
        this.lat2 = lat2;
    }

    /**
     * Gets longitude of point 2 (degrees).
     * @return longitude of point 2.
     */
    public double getLon2() {
        return lon2;
    }

    /**
     * Sets longitude of point 2 (degrees).
     * @param lon2 longitude of point 2.
     */
    public void setLon2(double lon2) {
        this.lon2 = lon2;
    }

    /**
     * Gets azimuth at point 2 (degrees).
     * @return azimuth at point 2.
     */
    public double getAzi2() {
        return azi2;
    }

    /**
     * Sets azimuth at point 2 (degrees).
     * @param azi2 azimuth at point 2.
     */
    public void setAzi2(double azi2) {
        this.azi2 = azi2;
    }

    /**
     * Gets distance between point 1 and point 2 (meters).
     * @return distance between point 1 and point 2.
     */
    public double getS12() {
        return s12;
    }

    /**
     * Sets distance between point 1 and point 2 (meters).
     * @param s12 distance between point 1 and point 2.
     */
    public void setS12(double s12) {
        this.s12 = s12;
    }

    /**
     * Gets arc length on the auxiliary sphere between point 1 and point 2 (degrees).
     * @return arc length on the auxiliary sphere between point 1 and point 2.
     */
    public double getA12() {
        return a12;
    }

    /**
     * Sets arc length on the auxiliary sphere between point 1 and point 2 (degrees).
     * @param a12 arc length on the auxiliary sphere between point 1 and point 2.
     */
    public void setA12(double a12) {
        this.a12 = a12;
    }

    /**
     * Gets reduced length of geodesic (meters).
     * @return reduced length of geodesic.
     */
    public double getM12() {
        return m12;
    }

    /**
     * Sets reduced length of geodesic (meters).
     * @param m12 reduced length of geodesic.
     */
    public void setM12(double m12) {
        this.m12 = m12;
    }

    /**
     * Gets geodesic scale of point 2 relative to point 1 (dimensionless).
     * @return geodesic scale of point 2 relative to point 1.
     */
    public double getScaleM12() {
        return scaleM12;
    }

    /**
     * Sets geodesic scale of point 2 relative to point 1 (dimensionless).
     * @param scaleM12 geodesic scale of point 2 relative to point 1.
     */
    public void setScaleM12(double scaleM12) {
        this.scaleM12 = scaleM12;
    }

    /**
     * Gets geodesic scale of point 1 relative to point 2 (dimensionless).
     * @return geodesic scale of point 1 relative to point 2.
     */
    public double getScaleM21() {
        return scaleM21;
    }

    /**
     * Sets geodesic scale of point 1 relative to point 2 (dimensionless).
     * @param scaleM21 geodesic scale of point 1 relative to point 2.
     */
    public void setScaleM21(double scaleM21) {
        this.scaleM21 = scaleM21;
    }

    /**
     * Gets area under the geodesic (meter<sup>2</sup>).
     * @return area under the geodesic.
     */
    public double getAreaS12() {
        return areaS12;
    }

    /**
     * Sets area under the geodesic (meter<sup>2</sup>).
     * @param areaS12 area under the geodesic.
     */
    public void setAreaS12(double areaS12) {
        this.areaS12 = areaS12;
    }
}
