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
 * A pair of double precision numbers.
 * This is like C++ class {@code std::pair<double, double>}.
 */
public class Pair {

    /**
     * The first member of the pair.
     */
    private double first;

    /**
     * The second member of the pair.
     */
    private double second;

    /**
     * Constructor.
     * @param first  the first member of the pair.
     * @param second the second member of the pair.
     */
    public Pair(double first, double second) {
        this.first = first;
        this.second = second;
    }

    /**
     * Gets the first member of the pair.
     * @return first member of the pair.
     */
    public double getFirst() {
        return first;
    }

    /**
     * Sets the first member of the pair.
     * @param first first member of the pair.
     */
    public void setFirst(double first) {
        this.first = first;
    }

    /**
     * Gets the second member of the pair.
     * @return second member of the pair.
     */
    public double getSecond() {
        return second;
    }

    /**
     * Sets the second member of the pair.
     * @param second second member of the pair.
     */
    public void setSecond(double second) {
        this.second = second;
    }
}
