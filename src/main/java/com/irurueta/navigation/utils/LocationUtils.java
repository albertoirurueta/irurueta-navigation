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
package com.irurueta.navigation.utils;

import com.irurueta.navigation.geodesic.Geodesic;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;

import java.text.DecimalFormat;
import java.util.StringTokenizer;

/**
 * Location utility class based on Android's SDK Location class.
 */
public class LocationUtils {
    /**
     * Constant used to specify formatting of a latitude or longitude
     * in the form "[+-]DDD.DDDDD where D indicates degrees.
     */
    public static final int FORMAT_DEGREES = 0;

    /**
     * Constant used to specify formatting of a latitude or longitude
     * in the form "[+-]DDD:MM.MMMMM" where D indicates degrees and
     * M indicates minutes of arc (1 minute = 1/60th of a degree).
     */
    public static final int FORMAT_MINUTES = 1;

    /**
     * Constant used to specify formatting of a latitude or longitude
     * in the form "DDD:MM:SS.SSSSS" where D indicates degrees, M
     * indicates minutes of arc, and S indicates seconds of arc (1
     * minute = 1/60th of a degree, 1 second = 1/3600th of a degree).
     */
    public static final int FORMAT_SECONDS = 2;

    /**
     * Constructor.
     * Prevents public instantiation.
     */
    private LocationUtils() {
    }

    /**
     * Converts a coordinate to a String representation. The outputType
     * may be one of FORMAT_DEGREES, FORMAT_MINUTES, or FORMAT_SECONDS.
     * The coordinate must be a valid double between -180.0 and 180.0.
     * This conversion is performed in a method that is dependent on the
     * default locale, and so is not guaranteed to round-trip with
     * {@link #convert(String)}.
     *
     * @param coordinate coordinate to be converted.
     * @param outputType output format.
     * @return converted coordinate.
     * @throws IllegalArgumentException if coordinate is less than
     *                                  -180.0, greater than 180.0, or is not a number.
     * @throws IllegalArgumentException if outputType is not one of
     *                                  FORMAT_DEGREES, FORMAT_MINUTES, or FORMAT_SECONDS.
     */
    public static String convert(double coordinate, final int outputType) {
        if (coordinate < -180.0 || coordinate > 180.0 || Double.isNaN(coordinate)) {
            throw new IllegalArgumentException();
        }
        if ((outputType != FORMAT_DEGREES) && (outputType != FORMAT_MINUTES) && (outputType != FORMAT_SECONDS)) {
            throw new IllegalArgumentException();
        }

        final var sb = new StringBuilder();

        // Handle negative values
        if (coordinate < 0) {
            sb.append('-');
            coordinate = -coordinate;
        }

        final var df = new DecimalFormat("###.#####");
        if (outputType == FORMAT_MINUTES || outputType == FORMAT_SECONDS) {
            final var degrees = (int) Math.floor(coordinate);
            sb.append(degrees);
            sb.append(':');
            coordinate -= degrees;
            coordinate *= 60.0;
            if (outputType == FORMAT_SECONDS) {
                final var minutes = (int) Math.floor(coordinate);
                sb.append(minutes);
                sb.append(':');
                coordinate -= minutes;
                coordinate *= 60.0;
            }
        }
        sb.append(df.format(coordinate));
        return sb.toString();
    }

    /**
     * Converts a String in one of the formats described by
     * FORMAT_DEGREES, FORMAT_MINUTES, or FORMAT_SECONDS into a
     * double. This conversion is performed in a locale agnostic
     * method, and so is not guaranteed to round-trip with
     * {@link #convert(double, int)}.
     *
     * @param coordinate coordinate to be parsed.
     * @return parsed value.
     * @throws NullPointerException     if coordinate is null
     * @throws IllegalArgumentException if the coordinate is not
     *                                  in one of the valid formats.
     */
    public static double convert(String coordinate) {
        // IllegalArgumentException if bad syntax
        if (coordinate == null) {
            throw new NullPointerException();
        }

        var negative = false;
        if (coordinate.charAt(0) == '-') {
            coordinate = coordinate.substring(1);
            negative = true;
        }

        final var st = new StringTokenizer(coordinate, ":");
        final var tokens = st.countTokens();
        if (tokens < 1) {
            throw new IllegalArgumentException();
        }
        try {
            final var degrees = st.nextToken();
            double val;
            if (tokens == 1) {
                val = Double.parseDouble(degrees);
                return negative ? -val : val;
            }

            final var minutes = st.nextToken();
            final var deg = Integer.parseInt(degrees);
            double min;
            var sec = 0.0;
            var secPresent = false;

            if (st.hasMoreTokens()) {
                min = Integer.parseInt(minutes);
                final var seconds = st.nextToken();
                sec = Double.parseDouble(seconds);
                secPresent = true;
            } else {
                min = Double.parseDouble(minutes);
            }

            final var isNegative180 = negative && (deg == 180) && (min == 0) && (sec == 0);

            // deg must be in [0, 179] except for the case of -180 degrees
            if ((deg < 0.0) || (deg > 179 && !isNegative180)) {
                throw new IllegalArgumentException();
            }

            // min must be in [0, 59] if seconds are present, otherwise [0.0, 60.0)
            if (min < 0 || min >= 60 || (secPresent && (min > 59))) {
                throw new IllegalArgumentException();
            }

            // sec must be in [0.0, 60.0)
            if (sec < 0 || sec >= 60) {
                throw new IllegalArgumentException();
            }

            val = deg * 3600.0 + min * 60.0 + sec;
            val /= 3600.0;
            return negative ? -val : val;
        } catch (final NumberFormatException nfe) {
            throw new IllegalArgumentException();
        }
    }

    /**
     * Computes the approximate distance in meters between two locations, and the initial and final bearings of the
     * shortest path between them. Distance and bearing are defined using the WGS84 ellipsoid.
     *
     * @param startLatitude  the starting latitude.
     * @param startLongitude the starting longitude.
     * @param endLatitude    the ending latitude.
     * @param endLongitude   the ending longitude.
     * @param results        instance containing results.
     */
    public static void distanceAndBearing(
            final double startLatitude, final double startLongitude, final double endLatitude,
            final double endLongitude, final BearingDistance results) {
        //noinspection all
        final var data = Geodesic.WGS84.inverse(startLatitude, startLongitude, endLatitude, endLongitude);
        results.startLatitude = data.getLat1();
        results.startLongitude = data.getLon1();
        results.endLatitude = data.getLat2();
        results.endLongitude = data.getLon2();
        results.distance = data.getS12();
        results.initialBearing = data.getAzi1();
        results.finalBearing = data.getAzi2();
    }

    /**
     * Computes the approximate distance in meters between two locations, and the initial and final bearings of the
     * shortest path between them. Distance and bearing are defined using the WGS84 ellipsoid.
     *
     * @param startLatitude  the starting latitude.
     * @param startLongitude the starting longitude.
     * @param endLatitude    the ending latitude.
     * @param endLongitude   the ending longitude.
     * @return bearing and distance results.
     */
    public static BearingDistance distanceAndBearing(
            final double startLatitude, final double startLongitude, final double endLatitude,
            final double endLongitude) {
        final var results = new BearingDistance();
        distanceAndBearing(startLatitude, startLongitude, endLatitude, endLongitude, results);
        return results;
    }

    /**
     * Computes the approximate distance in meters between two locations, and the initial and final bearings of the
     * shortest path between them. Distance and bearing are defined using the WGS84 ellipsoid.
     *
     * @param startLatitude  the starting latitude.
     * @param startLongitude the starting longitude.
     * @param endLatitude    the ending latitude.
     * @param endLongitude   the ending longitude.
     * @param results        array containing results. First element will contain distance. Second element will contain
     *                       initial bearing (optional). Third element will contain ending bearing (optional).
     * @throws IllegalArgumentException if results does not have at least 1 element.
     */
    public static void distanceAndBearing(
            final double startLatitude, final double startLongitude, final double endLatitude,
            final double endLongitude, final double[] results) {
        if (results.length == 0) {
            throw new IllegalArgumentException();
        }
        //noinspection all
        final var data = Geodesic.WGS84.inverse(startLatitude, startLongitude, endLatitude, endLongitude);
        results[0] = data.getS12();
        if (results.length > 1) {
            results[1] = data.getAzi1();
            if (results.length > 2) {
                results[2] = data.getAzi2();
            }
        }
    }

    /**
     * Computes the approximate distance in meters between two locations.
     *
     * @param startLatitude  the starting latitude.
     * @param startLongitude the starting longitude.
     * @param endLatitude    the ending latitude.
     * @param endLongitude   the ending longitude.
     * @return distance in meters between two locations.
     */
    public static double distanceBetweenMeters(
            final double startLatitude, final double startLongitude, final double endLatitude,
            final double endLongitude) {
        //noinspection all
        return Geodesic.WGS84.inverse(startLatitude, startLongitude, endLatitude, endLongitude).getS12();
    }

    /**
     * Computes the approximate distance between two locations.
     *
     * @param startLatitude  the starting latitude.
     * @param startLongitude the starting longitude.
     * @param endLatitude    the ending latitude.
     * @param endLongitude   the ending longitude.
     * @return distance between two locations.
     */
    public static Distance distanceBetween(
            final double startLatitude, final double startLongitude, final double endLatitude,
            final double endLongitude) {
        return new Distance(distanceBetweenMeters(startLatitude, startLongitude, endLatitude, endLongitude),
                DistanceUnit.METER);
    }

    /**
     * Computes the approximate distance between two locations.
     *
     * @param startLatitude  the starting latitude.
     * @param startLongitude the starting longitude.
     * @param endLatitude    the ending latitude.
     * @param endLongitude   the ending longitude.
     * @param result         instance where distance between two locations is stored.
     * @return provided result instance.
     */
    public static Distance distanceBetween(
            final double startLatitude, final double startLongitude, final double endLatitude,
            final double endLongitude, final Distance result) {
        result.setValue(distanceBetweenMeters(startLatitude, startLongitude, endLatitude, endLongitude));
        result.setUnit(DistanceUnit.METER);
        return result;
    }


    /**
     * Contains distance and bearing.
     */
    public static class BearingDistance {
        /**
         * Starting latitude (degrees).
         */
        private double startLatitude;

        /**
         * Starting longitude (degrees).
         */
        private double startLongitude;

        /**
         * Ending latitude (degrees).
         */
        private double endLatitude;

        /**
         * Ending longitude (degrees).
         */
        private double endLongitude;

        /**
         * Distance (meters).
         */
        private double distance = 0.0f;

        /**
         * Initial bearing (degrees).
         */
        private double initialBearing = 0.0f;

        /**
         * Final bearing (degrees).
         */
        private double finalBearing = 0.0f;

        /**
         * Gets starting latitude expressed in degrees.
         *
         * @return starting latitude (degrees).
         */
        public double getStartLatitude() {
            return startLatitude;
        }

        /**
         * Gets starting longitude expressed in degrees.
         *
         * @return starting longitude (degrees).
         */
        public double getStartLongitude() {
            return startLongitude;
        }

        /**
         * Gets ending latitude expressed in degrees.
         *
         * @return ending latitude (degrees).
         */
        public double getEndLatitude() {
            return endLatitude;
        }

        /**
         * Gets ending longitude expressed in degrees.
         *
         * @return ending longitude (degrees).
         */
        public double getEndLongitude() {
            return endLongitude;
        }

        /**
         * Gets distance expressed in meters.
         *
         * @return distance (meters).
         */
        public double getDistanceMeters() {
            return distance;
        }

        /**
         * Gets distance.
         *
         * @return distance.
         */
        public Distance getDistance() {
            return new Distance(distance, DistanceUnit.METER);
        }

        /**
         * Gets distance.
         *
         * @param result instance where result value is stored in meters.
         * @return provided distance instance.
         */
        public Distance getDistance(final Distance result) {
            result.setValue(distance);
            result.setUnit(DistanceUnit.METER);
            return result;
        }

        /**
         * Gets initial bearing/azimuth expressed in degrees.
         *
         * @return initial bearing/azimuth (degrees).
         */
        public double getInitialBearing() {
            return initialBearing;
        }

        /**
         * Gets final bearing/azimuth expressed in degrees.
         *
         * @return final bearing/azimuth (degrees).
         */
        public double getFinalBearing() {
            return finalBearing;
        }
    }
}
