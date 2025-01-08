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
import org.junit.jupiter.api.Test;

import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class Accuracy3DTest {

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final int TIMES = 50;

    @Test
    void testConstructor() throws AlgebraException, GeometryException {
        for (var t = 0; t < TIMES; t++) {
            // empty constructor
            var accuracy = new Accuracy3D();

            // check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertEquals(2.0, accuracy.getStandardDeviationFactor(), 0.0);
            assertEquals(0.9544, accuracy.getConfidence(), 1e-2);
            assertEquals(2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, accuracy.getConfidence(),
                    0.0);

            assertEquals(DistanceUnit.METER, accuracy.getSmallestAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getSmallestAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getSmallestAccuracyMeters(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getLargestAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getLargestAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getLargestAccuracyMeters(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getAverageAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getAverageAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getAverageAccuracyMeters(), 0.0);

            assertEquals(3, accuracy.getNumberOfDimensions());

            // constructor with covariance matrix
            final var randomizer = new UniformRandomizer();
            final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            var previous = 0.0;
            for (var i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
                semiAxesLengths[i] = previous + randomizer.nextDouble();
                previous = semiAxesLengths[i];
            }

            final var sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            for (var i = 0; i < Ellipsoid.DIMENSIONS; i++) {
                sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
            }

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

            final var ellipsoid = new Ellipsoid(Point3D.create(), semiAxesLengths, rotation);

            final var rotationMatrix = rotation.asInhomogeneousMatrix();
            final var covarianceMatrix = rotationMatrix.multiplyAndReturnNew(Matrix.diagonal(sqrSemiAxesLengths)
                    .multiplyAndReturnNew(rotationMatrix));

            accuracy = new Accuracy3D(covarianceMatrix);

            // check
            assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
            assertEquals(2.0, accuracy.getStandardDeviationFactor(), 0.0);
            assertEquals(0.9544, accuracy.getConfidence(), 1e-2);
            assertEquals(2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, accuracy.getConfidence(),
                    0.0);

            assertEquals(DistanceUnit.METER, accuracy.getSmallestAccuracy().getUnit());
            assertEquals(accuracy.getSmallestAccuracy().getValue().doubleValue(), accuracy.getSmallestAccuracyMeters(),
                    0.0);
            assertEquals(accuracy.getStandardDeviationFactor() * semiAxesLengths[2],
                    accuracy.getSmallestAccuracyMeters(), ABSOLUTE_ERROR);

            assertEquals(DistanceUnit.METER, accuracy.getLargestAccuracy().getUnit());
            assertEquals(accuracy.getLargestAccuracyMeters(), accuracy.getLargestAccuracy().getValue().doubleValue(),
                    0.0);
            assertEquals(accuracy.getStandardDeviationFactor() * semiAxesLengths[0],
                    accuracy.getLargestAccuracyMeters(), ABSOLUTE_ERROR);

            assertEquals(DistanceUnit.METER, accuracy.getAverageAccuracy().getUnit());
            assertEquals(accuracy.getAverageAccuracy().getValue().doubleValue(), accuracy.getAverageAccuracyMeters(),
                    0.0);
            assertEquals(accuracy.getStandardDeviationFactor()
                            * (semiAxesLengths[0] + semiAxesLengths[1] + semiAxesLengths[2]) / 3.0,
                    accuracy.getAverageAccuracyMeters(), ABSOLUTE_ERROR);

            assertEquals(3, accuracy.getNumberOfDimensions());

            var ellipsoid2 = accuracy.toEllipsoid();
            assertEquals(ellipsoid.getCenter(), ellipsoid2.getCenter());
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(ellipsoid.getSemiAxesLengths(),
                            accuracy.getStandardDeviationFactor()), ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid.getRotation().equals(ellipsoid2.getRotation(), ABSOLUTE_ERROR));
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(semiAxesLengths,
                    accuracy.getStandardDeviationFactor()), ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid2.getRotation().equals(rotation, ABSOLUTE_ERROR));

            // force IllegalArgumentException
            final var wrong = new Matrix(1, 1);
            assertThrows(IllegalArgumentException.class, () -> new Accuracy3D(wrong));

            // force NonSymmetricPositiveDefiniteMatrixException
            final var m = Matrix.diagonal(new double[]{
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
            assertThrows(NonSymmetricPositiveDefiniteMatrixException.class, () -> new Accuracy3D(m));

            // constructor with confidence
            final var conf = randomizer.nextDouble(0.0, 1.0);
            accuracy = new Accuracy3D(conf);

            // check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0),
                    accuracy.getStandardDeviationFactor(), 0.0);
            assertEquals(conf, accuracy.getConfidence(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getSmallestAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getSmallestAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getSmallestAccuracyMeters(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getLargestAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getLargestAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getLargestAccuracyMeters(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getAverageAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getAverageAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getAverageAccuracyMeters(), 0.0);

            assertEquals(3, accuracy.getNumberOfDimensions());

            // force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new Accuracy3D(-1.0));
            assertThrows(IllegalArgumentException.class, () -> new Accuracy3D(2.0));

            // constructor with covariance matrix and confidence
            accuracy = new Accuracy3D(covarianceMatrix, conf);

            // check
            assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(accuracy.getStandardDeviationFactor(),
                    NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
            assertEquals(accuracy.getConfidence(), conf, 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getSmallestAccuracy().getUnit());
            assertEquals(accuracy.getSmallestAccuracy().getValue().doubleValue(), accuracy.getSmallestAccuracyMeters(),
                    0.0);
            assertEquals(accuracy.getStandardDeviationFactor() * semiAxesLengths[2],
                    accuracy.getSmallestAccuracyMeters(), ABSOLUTE_ERROR);

            assertEquals(DistanceUnit.METER, accuracy.getLargestAccuracy().getUnit());
            assertEquals(accuracy.getLargestAccuracy().getValue().doubleValue(), accuracy.getLargestAccuracyMeters(),
                    0.0);
            assertEquals(accuracy.getStandardDeviationFactor() * semiAxesLengths[0],
                    accuracy.getLargestAccuracyMeters(), ABSOLUTE_ERROR);

            assertEquals(DistanceUnit.METER, accuracy.getAverageAccuracy().getUnit());
            assertEquals(accuracy.getAverageAccuracy().getValue().doubleValue(), accuracy.getAverageAccuracyMeters(),
                    0.0);
            assertEquals(accuracy.getStandardDeviationFactor()
                            * (semiAxesLengths[0] + semiAxesLengths[1] + semiAxesLengths[2]) / 3.0,
                    accuracy.getAverageAccuracyMeters(), ABSOLUTE_ERROR);

            assertEquals(3, accuracy.getNumberOfDimensions());

            ellipsoid2 = accuracy.toEllipsoid();
            assertEquals(ellipsoid.getCenter(), ellipsoid2.getCenter());
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(ellipsoid.getSemiAxesLengths(),
                            accuracy.getStandardDeviationFactor()), ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid.getRotation().equals(ellipsoid2.getRotation(), ABSOLUTE_ERROR));
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(semiAxesLengths,
                    accuracy.getStandardDeviationFactor()), ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid2.getRotation().equals(rotation, ABSOLUTE_ERROR));

            // force IllegalArgumentException
            assertThrows(IllegalArgumentException.class, () -> new Accuracy3D(wrong, conf));
            assertThrows(IllegalArgumentException.class, () -> new Accuracy3D(covarianceMatrix, -1.0));
            assertThrows(IllegalArgumentException.class, () -> new Accuracy3D(covarianceMatrix, 2.0));

            // force NonSymmetricPositiveDefiniteMatrixException
            assertThrows(NonSymmetricPositiveDefiniteMatrixException.class, () -> new Accuracy3D(m, conf));

            // test constructor with internal accuracy
            accuracy = new Accuracy3D(new com.irurueta.geometry.Accuracy3D(conf));

            // check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0),
                    accuracy.getStandardDeviationFactor(), 0.0);
            assertEquals(conf, accuracy.getConfidence(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getSmallestAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getSmallestAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getSmallestAccuracyMeters(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getLargestAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getLargestAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getLargestAccuracyMeters(), 0.0);

            assertEquals(DistanceUnit.METER, accuracy.getAverageAccuracy().getUnit());
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getAverageAccuracy().getValue().doubleValue(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, accuracy.getAverageAccuracyMeters(), 0.0);

            assertEquals(3, accuracy.getNumberOfDimensions());
        }
    }

    @Test
    void testGetSetCovarianceMatrix() throws AlgebraException {
        final var accuracy = new Accuracy3D();

        // check default value
        assertNull(accuracy.getCovarianceMatrix());

        // set new value
        final var covarianceMatrix = Matrix.identity(3, 3);
        accuracy.setCovarianceMatrix(covarianceMatrix);

        // check
        assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);

        // force IllegalArgumentException
        final var wrong = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> accuracy.setCovarianceMatrix(wrong));

        // force NonSymmetricPositiveDefiniteMatrixException
        final var m = Matrix.diagonal(new double[]{
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        assertThrows(NonSymmetricPositiveDefiniteMatrixException.class, () -> accuracy.setCovarianceMatrix(m));
    }

    @Test
    void testGetSetStandardDeviationFactor() {
        final var accuracy = new Accuracy3D();

        //check default value
        assertEquals(2.0, accuracy.getStandardDeviationFactor(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var factor = randomizer.nextDouble(10.0);
        accuracy.setStandardDeviationFactor(factor);

        // check
        assertEquals(factor, accuracy.getStandardDeviationFactor(), 0.0);
        assertEquals(2.0 * NormalDist.cdf(factor, 0.0, 1.0) - 1.0, accuracy.getConfidence(),
                0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> accuracy.setStandardDeviationFactor(0.0));
    }

    @Test
    void testGetSetConfidence() {
        final var accuracy = new Accuracy3D();

        // check default value
        assertEquals(0.9544, accuracy.getConfidence(), 1e-2);
        assertEquals(2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, accuracy.getConfidence(),
                0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var conf = randomizer.nextDouble();
        accuracy.setConfidence(conf);

        // check
        assertEquals(conf, accuracy.getConfidence(), 0.0);
        assertEquals(NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), accuracy.getStandardDeviationFactor(),
                0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> accuracy.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> accuracy.setConfidence(2.0));
    }

    @Test
    void testFlattenTo2D() throws AlgebraException, GeometryException {
        final var randomizer = new UniformRandomizer();
        final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        var previous = 0.0;
        for (var i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
            semiAxesLengths[i] = previous + randomizer.nextDouble();
            previous = semiAxesLengths[i];
        }

        final var sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        for (var i = 0; i < Ellipsoid.DIMENSIONS; i++) {
            sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
        }

        final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

        final var rotationMatrix = rotation.asInhomogeneousMatrix();
        final var covarianceMatrix = rotationMatrix.multiplyAndReturnNew(Matrix.diagonal(sqrSemiAxesLengths)
                .multiplyAndReturnNew(rotationMatrix));

        final var accuracy = new Accuracy3D(covarianceMatrix);

        final var flattenedAccuracy = accuracy.flattenTo2D();

        final var ellipse = accuracy.intersectWithPlane();
        final var flattenedEllipse = flattenedAccuracy.toEllipse();

        assertEquals(ellipse.getSemiMajorAxis(), flattenedEllipse.getSemiMajorAxis(), ABSOLUTE_ERROR);
        assertEquals(ellipse.getSemiMinorAxis(), flattenedEllipse.getSemiMinorAxis(), ABSOLUTE_ERROR);
        // because ellipses are symmetric, there is a rotation ambiguity
        assertTrue(Math.abs(ellipse.getRotationAngle()
                - flattenedEllipse.getRotationAngle()) <= ABSOLUTE_ERROR
                || Math.abs(Math.abs(ellipse.getRotationAngle()
                - flattenedEllipse.getRotationAngle()) - Math.PI) <= ABSOLUTE_ERROR);

        assertEquals(Point2D.create(), ellipse.getCenter());
        assertEquals(Point2D.create(), flattenedEllipse.getCenter());
    }

    @Test
    void testIntersectWithPlaneSphere() throws AlgebraException, GeometryException {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            Arrays.fill(sqrSemiAxesLengths, radius * radius);

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

            final var rotationMatrix = rotation.asInhomogeneousMatrix();
            final var covarianceMatrix = rotationMatrix.multiplyAndReturnNew(Matrix.diagonal(sqrSemiAxesLengths)
                    .multiplyAndReturnNew(rotationMatrix));

            final var accuracy = new com.irurueta.geometry.Accuracy3D(covarianceMatrix);

            final var ellipse = accuracy.intersectWithPlane();

            // check
            assertEquals(Point2D.create(), ellipse.getCenter());
            assertEquals(radius * accuracy.getStandardDeviationFactor(), ellipse.getSemiMajorAxis(),
                    ABSOLUTE_ERROR);
            assertEquals(radius * accuracy.getStandardDeviationFactor(), ellipse.getSemiMinorAxis(),
                    ABSOLUTE_ERROR);
        }
    }

    @Test
    void testIntersectWithPlaneEllipsoid() throws AlgebraException, GeometryException {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            var previous = 0.0;
            for (var i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
                semiAxesLengths[i] = previous + randomizer.nextDouble();
                previous = semiAxesLengths[i];
            }

            final var sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            for (var i = 0; i < Ellipsoid.DIMENSIONS; i++) {
                sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
            }

            final var roll = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

            final var rotationMatrix = rotation.asInhomogeneousMatrix();
            final var covarianceMatrix = rotationMatrix.multiplyAndReturnNew(Matrix.diagonal(sqrSemiAxesLengths)
                    .multiplyAndReturnNew(rotationMatrix));

            final var accuracy = new Accuracy3D(covarianceMatrix);
            accuracy.setStandardDeviationFactor(1.0);

            final var ellipse = accuracy.intersectWithPlane();

            // check
            assertEquals(Point2D.create(), ellipse.getCenter());
            assertTrue(ellipse.getSemiMajorAxis() <= semiAxesLengths[0]
                    && ellipse.getSemiMajorAxis() >= semiAxesLengths[2]);
            assertTrue(ellipse.getSemiMinorAxis() <= semiAxesLengths[0]
                    && ellipse.getSemiMinorAxis() >= semiAxesLengths[2]);
            assertTrue(ellipse.getSemiMajorAxis() >= ellipse.getSemiMinorAxis());
        }
    }

    @Test
    void testIntersectWithPlaneEllipsoidOnlyZAxisRotation() throws AlgebraException, GeometryException {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            var previous = 0.0;
            for (var i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
                semiAxesLengths[i] = previous + randomizer.nextDouble();
                previous = semiAxesLengths[i];
            }

            final var sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            for (var i = 0; i < Ellipsoid.DIMENSIONS; i++) {
                sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
            }

            final var angle = Utils.convertToRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var axis = new double[]{0.0, 0.0, 1.0};
            final var rotation = new AxisRotation3D(axis, angle);

            final var rotationMatrix = rotation.asInhomogeneousMatrix();
            final var covarianceMatrix = rotationMatrix.multiplyAndReturnNew(Matrix.diagonal(sqrSemiAxesLengths)
                    .multiplyAndReturnNew(rotationMatrix));

            final var accuracy = new Accuracy3D(covarianceMatrix);
            accuracy.setStandardDeviationFactor(1.0);

            final var ellipse = accuracy.intersectWithPlane();

            // check
            assertEquals(Point2D.create(), ellipse.getCenter());
            assertEquals(semiAxesLengths[0], ellipse.getSemiMajorAxis(), ABSOLUTE_ERROR);
            assertEquals(semiAxesLengths[1], ellipse.getSemiMinorAxis(), ABSOLUTE_ERROR);

            // because ellipses are symmetric, there is a rotation ambiguity
            assertTrue(Math.abs(ellipse.getRotationAngle() - angle) <= ABSOLUTE_ERROR
                    || Math.abs(Math.abs(ellipse.getRotationAngle() - angle) - Math.PI) <= ABSOLUTE_ERROR);
        }
    }
}
