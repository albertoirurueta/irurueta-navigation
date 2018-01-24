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
 * A container for the results from PolygonArea.
 */
public class PolygonResult {

    /**
     * The number of vertices in the polygon.
     */
    public int num;

    /**
     * The perimeter of the polygon or the length of the polyline (meters).
     */
    public double perimeter;

    /**
     * The area of the polygon (meters<sup>2</sup>).
     */
    public double area;

    /**
     * Constructor.
     * @param num the number of vertices in the polygon.
     * @param perimeter the perimeter of the polygon or the length of the polyline (meters).
     * @param area the area of the polygon (meters<sup>2</sup>).
     */
    @SuppressWarnings("WeakerAccess")
    public PolygonResult(int num, double perimeter, double area) {
        this.num = num;
        this.perimeter = perimeter;
        this.area = area;
    }
}
