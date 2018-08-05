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
package com.irurueta.navigation;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.*;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.DistanceUnit;
import org.junit.*;

import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class Accuracy3DTest {

    private static final Logger LOGGER = Logger.getLogger(Accuracy3DTest.class.getName());

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    @Test
    public void testConstructor() throws AlgebraException, GeometryException {
        for (int t = 0; t < TIMES; t++) {
            //empty constructor
            Accuracy3D accuracy = new Accuracy3D();

            //check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);
            assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
            assertEquals(accuracy.getConfidence(),
                    2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);

            assertEquals(accuracy.getSmallestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getSmallestAccuracy().getValue().doubleValue(),
                    Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getSmallestAccuracyMeters(), Double.POSITIVE_INFINITY, 0.0);

            assertEquals(accuracy.getLargestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getLargestAccuracy().getValue().doubleValue(),
                    Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getLargestAccuracyMeters(), Double.POSITIVE_INFINITY, 0.0);

            assertEquals(accuracy.getAverageAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getAverageAccuracy().getValue().doubleValue(),
                    Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getAverageAccuracyMeters(), Double.POSITIVE_INFINITY, 0.0);

            assertEquals(accuracy.getNumberOfDimensions(), 3);


            //constructor with covariance matrix
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            double previous = 0.0;
            for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
                semiAxesLengths[i] = previous + randomizer.nextDouble();
                previous = semiAxesLengths[i];
            }
            double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

            Ellipsoid ellipsoid = new Ellipsoid(Point3D.create(), semiAxesLengths, rotation);

            Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
            Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                    Matrix.diagonal(semiAxesLengths).multiplyAndReturnNew(rotationMatrix));

            accuracy = new Accuracy3D(covarianceMatrix);

            //check
            assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
            assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);
            assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
            assertEquals(accuracy.getConfidence(),
                    2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);

            assertEquals(accuracy.getSmallestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getSmallestAccuracy().getValue().doubleValue(),
                    accuracy.getSmallestAccuracyMeters(), 0.0);
            assertEquals(accuracy.getSmallestAccuracyMeters(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[2],
                    ABSOLUTE_ERROR);

            assertEquals(accuracy.getLargestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getLargestAccuracy().getValue().doubleValue(),
                    accuracy.getLargestAccuracyMeters(), 0.0);
            assertEquals(accuracy.getLargestAccuracyMeters(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[0],
                    ABSOLUTE_ERROR);

            assertEquals(accuracy.getAverageAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getAverageAccuracy().getValue().doubleValue(),
                    accuracy.getAverageAccuracyMeters(), 0.0);
            assertEquals(accuracy.getAverageAccuracyMeters(), accuracy.getStandardDeviationFactor() *
                            (semiAxesLengths[0] + semiAxesLengths[1] + semiAxesLengths[2]) / 3.0, ABSOLUTE_ERROR);

            assertEquals(accuracy.getNumberOfDimensions(), 3);

            Ellipsoid ellipsoid2 = accuracy.toEllipsoid();
            assertEquals(ellipsoid.getCenter(), ellipsoid2.getCenter());
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(
                    ellipsoid.getSemiAxesLengths(), accuracy.getStandardDeviationFactor()),
                    ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid.getRotation().equals(ellipsoid2.getRotation(), ABSOLUTE_ERROR));
            assertArrayEquals(ellipsoid2.getSemiAxesLengths(),
                    ArrayUtils.multiplyByScalarAndReturnNew(semiAxesLengths,
                            accuracy.getStandardDeviationFactor()), ABSOLUTE_ERROR);
            assertTrue(ellipsoid2.getRotation().equals(rotation, ABSOLUTE_ERROR));

            //force IllegalArgumentException
            accuracy = null;
            try {
                accuracy = new Accuracy3D(new Matrix(1, 1));
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(accuracy);

            //force NonSymmetricPositiveDefiniteMatrixException
            Matrix m = Matrix.diagonal(new double[]{
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});

            try {
                accuracy = new Accuracy3D(m);
                fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) { }
            assertNull(accuracy);


            //constructor with confidence
            double conf = randomizer.nextDouble(0.0, 1.0);
            accuracy = new Accuracy3D(conf);

            //check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(accuracy.getStandardDeviationFactor(),
                    NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
            assertEquals(accuracy.getConfidence(), conf, 0.0);

            assertEquals(accuracy.getSmallestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getSmallestAccuracy().getValue().doubleValue(),
                    Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getSmallestAccuracyMeters(), Double.POSITIVE_INFINITY, 0.0);

            assertEquals(accuracy.getLargestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getLargestAccuracy().getValue().doubleValue(),
                    Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getLargestAccuracyMeters(), Double.POSITIVE_INFINITY, 0.0);

            assertEquals(accuracy.getAverageAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getAverageAccuracy().getValue().doubleValue(),
                    Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getAverageAccuracyMeters(), Double.POSITIVE_INFINITY, 0.0);

            assertEquals(accuracy.getNumberOfDimensions(), 3);

            //force IllegalArgumentException
            accuracy = null;
            try {
                accuracy = new Accuracy3D(-1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                accuracy = new Accuracy3D(2.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(accuracy);


            //constructor with covariance matrix and confidence
            accuracy = new Accuracy3D(covarianceMatrix, conf);

            //check
            assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(accuracy.getStandardDeviationFactor(),
                    NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
            assertEquals(accuracy.getConfidence(), conf, 0.0);

            assertEquals(accuracy.getSmallestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getSmallestAccuracy().getValue().doubleValue(),
                    accuracy.getSmallestAccuracyMeters(), 0.0);
            assertEquals(accuracy.getSmallestAccuracyMeters(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[2],
                    ABSOLUTE_ERROR);

            assertEquals(accuracy.getLargestAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getLargestAccuracy().getValue().doubleValue(),
                    accuracy.getLargestAccuracyMeters(), 0.0);
            assertEquals(accuracy.getLargestAccuracyMeters(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[0],
                    ABSOLUTE_ERROR);

            assertEquals(accuracy.getAverageAccuracy().getUnit(), DistanceUnit.METER);
            assertEquals(accuracy.getAverageAccuracy().getValue().doubleValue(),
                    accuracy.getAverageAccuracyMeters(), 0.0);
            assertEquals(accuracy.getAverageAccuracyMeters(), accuracy.getStandardDeviationFactor() *
                    (semiAxesLengths[0] + semiAxesLengths[1] + semiAxesLengths[2]) / 3.0,
                    ABSOLUTE_ERROR);

            assertEquals(accuracy.getNumberOfDimensions(), 3);

            ellipsoid2 = accuracy.toEllipsoid();
            assertEquals(ellipsoid.getCenter(), ellipsoid2.getCenter());
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(
                    ellipsoid.getSemiAxesLengths(), accuracy.getStandardDeviationFactor()),
                    ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid.getRotation().equals(ellipsoid2.getRotation(), ABSOLUTE_ERROR));
            assertArrayEquals(ellipsoid2.getSemiAxesLengths(),
                    ArrayUtils.multiplyByScalarAndReturnNew(semiAxesLengths,
                            accuracy.getStandardDeviationFactor()), ABSOLUTE_ERROR);
            assertTrue(ellipsoid2.getRotation().equals(rotation, ABSOLUTE_ERROR));


            //force IllegalArgumentException
            accuracy = null;
            try {
                accuracy = new Accuracy3D(new Matrix(1, 1), conf);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                accuracy = new Accuracy3D(covarianceMatrix, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            try {
                accuracy = new Accuracy3D(covarianceMatrix, 2.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(accuracy);

            //force NonSymmetricPositiveDefiniteMatrixException
            try {
                accuracy = new Accuracy3D(m, conf);
                fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) { }
            assertNull(accuracy);
        }
    }

    @Test
    public void testGetSetCovarianceMatrix() throws AlgebraException {
        Accuracy3D accuracy = new Accuracy3D();

        //check default value
        assertNull(accuracy.getCovarianceMatrix());

        //set new value
        Matrix covarianceMatrix = Matrix.identity(3, 3);
        accuracy.setCovarianceMatrix(covarianceMatrix);

        //check
        assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);

        //force IllegalArgumentException
        try {
            accuracy.setCovarianceMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        //force NonSymmetricPositiveDefiniteMatrixException
        Matrix m = Matrix.diagonal(new double[]{
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        try {
            accuracy.setCovarianceMatrix(m);
            fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
        } catch (NonSymmetricPositiveDefiniteMatrixException ignore) { }
    }

    @Test
    public void testGetSetStandardDeviationFactor() {
        Accuracy3D accuracy = new Accuracy3D();

        //check default value
        assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double factor = randomizer.nextDouble(10.0);
        accuracy.setStandardDeviationFactor(factor);

        //check
        assertEquals(accuracy.getStandardDeviationFactor(), factor, 0.0);
        assertEquals(accuracy.getConfidence(),
                2.0 * NormalDist.cdf(factor, 0.0, 1.0) - 1.0, 0.0);

        //force IllegalArgumentException
        try {
            accuracy.setStandardDeviationFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetConfidence() {
        Accuracy3D accuracy = new Accuracy3D();

        //check default value
        assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
        assertEquals(accuracy.getConfidence(),
                2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double conf = randomizer.nextDouble();
        accuracy.setConfidence(conf);

        //check
        assertEquals(accuracy.getConfidence(), conf, 0.0);
        assertEquals(accuracy.getStandardDeviationFactor(),
                NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);

        //force IllegalArgumentException
        try {
            accuracy.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            accuracy.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testFlattenTo2D() throws AlgebraException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        double previous = 0.0;
        for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
            semiAxesLengths[i] = previous + randomizer.nextDouble();
            previous = semiAxesLengths[i];
        }
        double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

        Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(semiAxesLengths).multiplyAndReturnNew(rotationMatrix));

        Accuracy3D accuracy = new Accuracy3D(covarianceMatrix);

        Accuracy2D flattenedAccuracy = accuracy.flattenTo2D();

        assertEquals(flattenedAccuracy.getCovarianceMatrix(),
                covarianceMatrix.getSubmatrix(0, 0,
                        1, 1));
        assertEquals(flattenedAccuracy.getStandardDeviationFactor(),
                accuracy.getStandardDeviationFactor(), 0.0);
        assertEquals(flattenedAccuracy.getConfidence(), accuracy.getConfidence(), 0.0);

        LOGGER.log(Level.INFO, "Avg accuracy 2D: {0} m",
                flattenedAccuracy.getAverageAccuracy());
        LOGGER.log(Level.INFO, "Avg accuracy 3D: {0} m",
                accuracy.getAverageAccuracy());
    }
}
