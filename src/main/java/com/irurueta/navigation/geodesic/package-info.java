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

/**
 * Contains geodesic algorithms.
 * Makes easy to do geodesic computations for an ellipsoid of revolution.
 * The contents of this package are based on geographicLib implementation
 * <a href="https://sourceforge.net/projects/geographiclib/">Geographiclib</a>
 *
 * <p>
 * The important classes are:
 * <ul>
 *     <li>
 *         {@link com.irurueta.navigation.geodesic.Geodesic}, for direct and inverse geodesic calculations.
 *     </li>
 *     <li>
 *         {@link com.irurueta.navigation.geodesic.GeodesicLine}, an efficient way of calculating multiple points on a
 *         single geodesic.
 *     </li>
 *     <li>
 *         {@link com.irurueta.navigation.geodesic.GeodesicData}, object containing the results of
 *         geodesic calculations.
 *     </li>
 *     <li>
 *         {@link com.irurueta.navigation.geodesic.GeodesicMask} constants that let you specify
 *         the variables to return in {@link com.irurueta.navigation.geodesic.GeodesicData} and
 *         the capabilities of a {@link com.irurueta.navigation.geodesic.GeodesicLine}.
 *     </li>
 *     <li>
 *         {@link com.irurueta.navigation.geodesic.Constants} the parameters for the WGS84 ellipsoid.
 *     </li>
 *     <li>
 *         {@link com.irurueta.navigation.geodesic.PolygonArea}, a class to compute the perimeter and area of a geodesic
 *         polygon (returned as a {@link com.irurueta.navigation.geodesic.PolygonResult}).
 *     </li>
 * </ul>
 *
 * <h2>External links</h2>
 * <ul>
 *     <li>
 *         These algorithms are derived in C.F.F. Karney, <a href="https://doi.org/10.1007/s00190-012-0578-z">
 *         Algorithms for geodesics</a>, J.Geodesy <b>87</b>, 43&ndash;55 (2013)
 *         (<a href="https://geographiclib.sourceforge.io/geod-addenda.html">addenda</a>
 *     </li>
 *     <li>
 *         A longer paper on geodesics: C.F.F. Karney,
 *         <a href="https://arxic.org/abs/1102.1215v1">Geodesics on an ellipsoid of revolution</a>,
 *         Feb. 2011 (<a href="https://geographiclib.sourceforge.io/geod-addenda.html#geod-errata">
 *         errata</a>).
 *     </li>
 *     <li>
 *         <a href="https://geographiclib.sourceforge.io">The GeographicLib web site</a>.
 *     </li>
 *     <li>
 *         The wikipedia page,
 *         <a href="https://en.wikipedia.org/wiki/Geodesics_on_an_ellipsoid">
 *         Geodesics on an ellipsoid</a>
 *     </li>
 * </ul>
 */
package com.irurueta.navigation.geodesic;