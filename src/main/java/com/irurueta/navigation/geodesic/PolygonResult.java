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
    private int num;

    /**
     * The perimeter of the polygon or the length of the polyline (meters).
     */
    private double perimeter;

    /**
     * The area of the polygon (meters<sup>2</sup>).
     */
    private double area;

    /**
     * Constructor.
     *
     * @param num       the number of vertices in the polygon.
     * @param perimeter the perimeter of the polygon or the length of the polyline (meters).
     * @param area      the area of the polygon (meters<sup>2</sup>).
     */
    public PolygonResult(final int num, final double perimeter, final double area) {
        this.num = num;
        this.perimeter = perimeter;
        this.area = area;
    }

    /**
     * Gets the number of vertices in the polygon.
     *
     * @return number of vertices in the polygon.
     */
    public int getNum() {
        return num;
    }

    /**
     * Sets the number of vertices in the polygon.
     *
     * @param num number of vertices in the polygon.
     */
    public void setNum(final int num) {
        this.num = num;
    }

    /**
     * Gets the perimeter of the polygon or the length of the polyline (meters).
     *
     * @return the perimeter of the polygon or the length of the polyline.
     */
    public double getPerimeter() {
        return perimeter;
    }

    /**
     * Sets the perimeter of the polygon or the length of the polyline (meters).
     *
     * @param perimeter the perimeter of the polygon or the length of the polyline.
     */
    public void setPerimeter(final double perimeter) {
        this.perimeter = perimeter;
    }

    /**
     * Gets the area of the polygon (meters<sup>2</sup>).
     *
     * @return area of the polygon.
     */
    public double getArea() {
        return area;
    }

    /**
     * Sets the area of the polygon (meters<sup>2</sup>).
     *
     * @param area area of the polygon.
     */
    public void setArea(double area) {
        this.area = area;
    }
}
