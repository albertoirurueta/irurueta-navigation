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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class MagneticFluxDensityTriadTest {

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.getUnit());
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);
        final double[] values = new double[3];
        triad.getValuesAsArray(values);
        assertArrayEquals(new double[3], values, 0.0);
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());
        final Matrix v = new Matrix(3, 1);
        triad.getValuesAsMatrix(v);
        assertEquals(new Matrix(3, 1), v);
        final MagneticFluxDensity vx1 = triad.getMeasurementX();
        assertEquals(0.0, vx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx1.getUnit());
        final MagneticFluxDensity vx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx2.getUnit());
        final MagneticFluxDensity vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy1.getUnit());
        final MagneticFluxDensity vy2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy2.getUnit());
        final MagneticFluxDensity vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz1.getUnit());
        final MagneticFluxDensity vz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz2.getUnit());
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.NANOTESLA);

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);
        final double[] values = new double[3];
        triad.getValuesAsArray(values);
        assertArrayEquals(new double[3], values, 0.0);
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());
        final Matrix v = new Matrix(3, 1);
        triad.getValuesAsMatrix(v);
        assertEquals(new Matrix(3, 1), v);
        final MagneticFluxDensity vx1 = triad.getMeasurementX();
        assertEquals(0.0, vx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx1.getUnit());
        final MagneticFluxDensity vx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx2.getUnit());
        final MagneticFluxDensity vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy1.getUnit());
        final MagneticFluxDensity vy2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy2.getUnit());
        final MagneticFluxDensity vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz1.getUnit());
        final MagneticFluxDensity vz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz2.getUnit());
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final double[] values1 = new double[]{ valueX, valueY, valueZ };
        final Matrix v1 = Matrix.newFromArray(values1);

        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityTriad.DEFAULT_UNIT, triad.getUnit());
        assertArrayEquals(new double[]{ valueX, valueY, valueZ },
                triad.getValuesAsArray(), 0.0);
        final double[] values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final Matrix v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final MagneticFluxDensity vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx1.getUnit());
        final MagneticFluxDensity vx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx2.getUnit());
        final MagneticFluxDensity vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy1.getUnit());
        final MagneticFluxDensity vy2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy2.getUnit());
        final MagneticFluxDensity vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz1.getUnit());
        final MagneticFluxDensity vz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz2.getUnit());
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final double[] values1 = new double[]{ valueX, valueY, valueZ };
        final Matrix v1 = Matrix.newFromArray(values1);

        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.NANOTESLA, valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());
        assertArrayEquals(new double[]{ valueX, valueY, valueZ },
                triad.getValuesAsArray(), 0.0);
        final double[] values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final Matrix v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final MagneticFluxDensity vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx1.getUnit());
        final MagneticFluxDensity vx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx2.getUnit());
        final MagneticFluxDensity vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy1.getUnit());
        final MagneticFluxDensity vy2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy2.getUnit());
        final MagneticFluxDensity vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz1.getUnit());
        final MagneticFluxDensity vz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz2.getUnit());
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensity bx = new MagneticFluxDensity(
                valueX, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity by = new MagneticFluxDensity(
                valueY, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bz = new MagneticFluxDensity(
                valueZ, MagneticFluxDensityUnit.TESLA);

        final double[] values1 = new double[]{ valueX, valueY, valueZ };
        final Matrix v1 = Matrix.newFromArray(values1);

        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(bx, by, bz);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityTriad.DEFAULT_UNIT, triad.getUnit());
        assertArrayEquals(new double[]{ valueX, valueY, valueZ },
                triad.getValuesAsArray(), 0.0);
        final double[] values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final Matrix v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final MagneticFluxDensity vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx1.getUnit());
        final MagneticFluxDensity vx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx2.getUnit());
        final MagneticFluxDensity vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy1.getUnit());
        final MagneticFluxDensity vy2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy2.getUnit());
        final MagneticFluxDensity vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz1.getUnit());
        final MagneticFluxDensity vz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz2.getUnit());
    }

    @Test
    public void testConstructor6() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(triad1);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(triad1.getUnit(), triad2.getUnit());
    }

    @Test
    public void testGetSetValueX() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(0.0, triad.getValueX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();

        triad.setValueX(valueX);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
    }

    @Test
    public void testGetSetValueY() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(0.0, triad.getValueY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueY = randomizer.nextDouble();

        triad.setValueY(valueY);

        // check
        assertEquals(valueY, triad.getValueY(), 0.0);
    }

    @Test
    public void testGetSetValueZ() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueZ = randomizer.nextDouble();

        triad.setValueZ(valueZ);

        // check
        assertEquals(valueZ, triad.getValueZ(), 0.0);
    }

    @Test
    public void testSetValueCoordinates() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        triad.setValueCoordinates(valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
    }

    @Test
    public void testGetSetUnit() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.getUnit());

        // set new value
        triad.setUnit(MagneticFluxDensityUnit.NANOTESLA);

        // check
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());

        // Force IllegalArgumentException
        try {
            triad.setUnit(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetValueCoordinatesAndUnit() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default values
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ,
                MagneticFluxDensityUnit.NANOTESLA);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());

        // Force IllegalArgumentException
        try {
            triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetValuesAsArray() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertArrayEquals(new double[3], triad.getValuesAsArray(),
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] values1 = new double[3];
        randomizer.fill(values1);

        triad.setValueCoordinates(values1);

        // check
        final double[] values2 = triad.getValuesAsArray();
        final double[] values3 = new double[3];
        triad.getValuesAsArray(values3);

        assertArrayEquals(values1, values2, 0.0);
        assertArrayEquals(values1, values3, 0.0);

        // Force IllegalArgumentException
        try {
            triad.getValuesAsArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triad.setValueCoordinates(new double[2]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetValuesAsMatrix() throws WrongSizeException {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(new Matrix(3, 1),
                triad.getValuesAsMatrix());

        // set new value
        final Matrix values1 = Matrix.createWithUniformRandomValues(
                3, 1, -1.0, 1.0);

        triad.setValueCoordinates(values1);

        // check
        final Matrix values2 = triad.getValuesAsMatrix();
        final Matrix values3 = new Matrix(3, 1);
        triad.getValuesAsMatrix(values3);

        assertEquals(values1, values2);
        assertEquals(values1, values3);

        // Force IllegalArgumentException
        try {
            triad.getValuesAsMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triad.getValuesAsMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triad.setValueCoordinates(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            triad.setValueCoordinates(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMeasurementX() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        final MagneticFluxDensity bx1 = triad.getMeasurementX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                valueX, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementX(bx2);

        // check
        final MagneticFluxDensity bx3 = triad.getMeasurementX();
        final MagneticFluxDensity bx4 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementX(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetMeasurementY() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        final MagneticFluxDensity by1 = triad.getMeasurementY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueY = randomizer.nextDouble();
        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                valueY, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementY(by2);

        // check
        final MagneticFluxDensity by3 = triad.getMeasurementY();
        final MagneticFluxDensity by4 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementY(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetMeasurementZ() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default value
        final MagneticFluxDensity bz1 = triad.getMeasurementZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueZ = randomizer.nextDouble();
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                valueZ, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementZ(bz2);

        // chekc
        final MagneticFluxDensity bz3 = triad.getMeasurementZ();
        final MagneticFluxDensity bz4 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementZ(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testSetMeasurementCoordinates() {
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();

        // check default values
        final MagneticFluxDensity bx1 = triad.getMeasurementX();
        final MagneticFluxDensity by1 = triad.getMeasurementY();
        final MagneticFluxDensity bz1 = triad.getMeasurementZ();

        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                valueX, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                valueY, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                valueZ, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementCoordinates(bx2, by2, bz2);

        // check
        final MagneticFluxDensity bx3 = triad.getMeasurementX();
        final MagneticFluxDensity by3 = triad.getMeasurementY();
        final MagneticFluxDensity bz3 = triad.getMeasurementZ();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.NANOTESLA);

        triad1.copyTo(triad2);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad2.getUnit());
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.NANOTESLA);

        triad2.copyFrom(triad1);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad2.getUnit());
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(triad1);
        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad();

        assertEquals(triad1.hashCode(), triad2.hashCode());
        assertNotEquals(triad1.hashCode(), triad3.hashCode());
    }

    @Test
    public void testEquals1() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(triad1);
        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad();

        assertTrue(triad1.equals(triad2));
        assertTrue(triad2.equals(triad1));
        assertTrue(triad1.equals(triad1));
        assertTrue(triad2.equals(triad2));
        assertFalse(triad1.equals(triad3));
        assertFalse(triad2.equals(triad3));
        assertFalse(triad1.equals(null));
    }

    @Test
    public void testEquals2() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(triad1);
        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad();

        assertTrue(triad1.equals(triad2, ABSOLUTE_ERROR));
        assertTrue(triad2.equals(triad1, ABSOLUTE_ERROR));
        assertTrue(triad1.equals(triad1, ABSOLUTE_ERROR));
        assertTrue(triad2.equals(triad2, ABSOLUTE_ERROR));
        assertFalse(triad1.equals(triad3, ABSOLUTE_ERROR));
        assertFalse(triad2.equals(triad3, ABSOLUTE_ERROR));
        assertFalse(triad1.equals(null, ABSOLUTE_ERROR));
    }

    @Test
    public void testEquals3() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(triad1);
        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad();
        final Object obj = new Object();

        assertEquals(triad1, triad2);
        assertNotEquals(triad1, triad3);
        assertNotEquals(triad1, obj);
        assertNotEquals(triad1, null);
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad1 = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);
        final MagneticFluxDensityTriad triad2 = (MagneticFluxDensityTriad) triad1.clone();

        assertEquals(triad1, triad2);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad(
                valueX, valueY, valueZ);

        final double sqrNorm = valueX * valueX + valueY * valueY + valueZ * valueZ;
        final double norm = Math.sqrt(sqrNorm);

        assertEquals(sqrNorm, triad.getSqrNorm(), 0.0);
        assertEquals(norm, triad.getNorm(), 0.0);

        final MagneticFluxDensity b1 = triad.getMeasurementNorm();
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementNorm(b2);

        assertEquals(norm, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        assertEquals(b1, b2);
    }
}
