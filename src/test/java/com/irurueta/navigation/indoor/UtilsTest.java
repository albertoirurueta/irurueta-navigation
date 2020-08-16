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
package com.irurueta.navigation.indoor;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.numerical.DerivativeEstimator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

@SuppressWarnings("Duplicates")
public class UtilsTest {

    private static final Logger LOGGER = Logger.getLogger(
            UtilsTest.class.getName());


    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double MIN_RSSI = -100.0;
    private static final double MAX_RSSI = 100.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;
    private static final double MAX_PATH_LOSS_EXPONENT = 2.0;

    private static final double MIN_DISTANCE = 0.5;
    private static final double MAX_DISTANCE = 50.0;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double TX_POWER_VARIANCE = 0.1;
    private static final double RX_POWER_VARIANCE = 0.5;
    private static final double PATHLOSS_EXPONENT_VARIANCE = 0.001;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 50;

    public UtilsTest() {
    }

    @BeforeClass
    public static void setUpClass() {
    }

    @AfterClass
    public static void tearDownClass() {
    }

    @Before
    public void setUp() {
    }

    @After
    public void tearDown() {
    }

    @Test
    public void testdBmToPowerAndPowerTodBm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double value = randomizer.nextDouble();

        assertEquals(Utils.powerTodBm(Utils.dBmToPower(value)), value,
                ABSOLUTE_ERROR);
        assertEquals(Utils.dBmToPower(Utils.powerTodBm(value)), value,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testPropagatePowerVarianceToDistanceVariance()
            throws EvaluationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            double distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    RX_POWER_VARIANCE);
            assertTrue(distanceVariance > 0.0);

            final double k = SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
            final double kdB = 10.0 * Math.log10(k);
            final double derivative = -Math.log(10.0) / (10.0 * pathLossExponent) *
                    Math.pow(10.0, (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) /
                            (10.0 * pathLossExponent));

            DerivativeEstimator derivativeEstimator = new DerivativeEstimator(
                    new SingleDimensionFunctionEvaluatorListener() {
                        @Override
                        public double evaluate(double point) {
                            final double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
                            final double kdB = 10.0 * Math.log10(k);
                            final double logSqrDistance = (pathLossExponent * kdB + txPowerdBm - point) / (5.0 * pathLossExponent);
                            return Math.pow(10.0, logSqrDistance / 2.0);
                        }
                    });

            final double derivative2 = derivativeEstimator.derivative(rxPowerdBm);
            assertEquals(derivative, derivative2, LARGE_ABSOLUTE_ERROR);

            assertEquals(distanceVariance,
                    derivative * derivative * RX_POWER_VARIANCE, 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;

            // check that if rx power variance is zero, then distance variance is
            // zero as well
            distanceVariance = Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0);
            assertEquals(distanceVariance, 0.0, 0.0);

            // test without rx power
            assertEquals(Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    null), 0.0, 0.0);
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateVariancesToDistanceVariance()
            throws IndoorException, EvaluationException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            final MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    TX_POWER_VARIANCE, RX_POWER_VARIANCE, PATHLOSS_EXPONENT_VARIANCE);

            final double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY);
            final double kdB = 10.0 * Math.log10(k);

            final double tenPathLossExponent = 10.0 * pathLossExponent;
            final double tmp = (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / tenPathLossExponent;
            final double derivativeTxPower = Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                    tmp);
            final double derivativeRxPower = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                    tmp);

            final double g = (pathLossExponent * kdB + txPowerdBm - rxPowerdBm) / (10.0 * pathLossExponent);
            final double derivativeG = (kdB * 10.0 * pathLossExponent -
                    10.0 * (pathLossExponent * kdB + txPowerdBm - rxPowerdBm)) /
                    Math.pow(10.0 * pathLossExponent, 2.0);

            final double derivativePathLossExponent = Math.log(10.0) * derivativeG * Math.pow(10.0, g);

            final double[] gradient = new double[]{
                    derivativeTxPower,
                    derivativeRxPower,
                    derivativePathLossExponent
            };

            final GradientEstimator gradientEstimator = new GradientEstimator(
                    new MultiDimensionFunctionEvaluatorListener() {
                        @Override
                        public double evaluate(double[] point) {
                            final double txPower = point[0];
                            final double rxPower = point[1];
                            final double pathLossExponent = point[2];

                            final double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower)
                                    / (5.0 * pathLossExponent);
                            return Math.pow(10.0, logSqrDistance / 2.0);
                        }
                    });

            final double[] gradient2 = gradientEstimator.gradient(
                    new double[]{
                            txPowerdBm,
                            rxPowerdBm,
                            pathLossExponent});

            assertArrayEquals(gradient, gradient2, LARGE_ABSOLUTE_ERROR);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;

            // check without variances
            assertNull(Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    null, null, null));
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateTxPowerVarianceToDistanceVariance() throws IndoorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            final MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    TX_POWER_VARIANCE, 0.0, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateRxPowerVarianceToDistanceVariance() throws IndoorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            final MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0, RX_POWER_VARIANCE, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            final double distanceVariance2 = Utils.propagatePowerVarianceToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    RX_POWER_VARIANCE);
            assertEquals(distanceVariance, distanceVariance2, ABSOLUTE_ERROR);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagatePathlossExponentVarianceToDistanceVariance() throws IndoorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double minDistanceVariance = Double.MAX_VALUE;
        double maxDistanceVariance = -Double.MAX_VALUE;
        double avgDistanceVariance = 0.0;
        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            final MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0, 0.0, PATHLOSS_EXPONENT_VARIANCE);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertTrue(distanceVariance > 0.0);

            if (distanceVariance < minDistanceVariance) {
                minDistanceVariance = distanceVariance;
            }
            if (distanceVariance > maxDistanceVariance) {
                maxDistanceVariance = distanceVariance;
            }
            avgDistanceVariance += distanceVariance / TIMES;
        }

        LOGGER.log(Level.INFO, "Min dist variance: {0}", minDistanceVariance);
        LOGGER.log(Level.INFO, "Max dist variance: {0}", maxDistanceVariance);
        LOGGER.log(Level.INFO, "Avg dist variance: {0}", avgDistanceVariance);
    }

    @Test
    public void testPropagateNoVarianceToDistanceVariance() throws IndoorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double txPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double txPower = Utils.dBmToPower(txPowerdBm);

            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);
            final double distance = randomizer.nextDouble(MIN_DISTANCE, MAX_DISTANCE);

            final double rxPowerdBm = Utils.powerTodBm(
                    receivedPower(txPower, distance,
                            pathLossExponent));

            final MultivariateNormalDist dist = Utils.propagateVariancesToDistanceVariance(
                    txPowerdBm, rxPowerdBm, pathLossExponent, FREQUENCY,
                    0.0, 0.0, 0.0);

            assertEquals(dist.getMean()[0], distance, ABSOLUTE_ERROR);

            final double distanceVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(distanceVariance, 0.0, ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceFirstOrderNonLinear2D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;

            final double diffXi1 = xi - x1;
            final double diffYi1 = yi - y1;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;

            final double d1a2 = diffX1a2 + diffY1a2;

            final double rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1) / (Math.log(10.0) * d1a2);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2, 2),
                    new Matrix(2, 2), new Matrix(2, 2));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceFirstOrderNonLinear3D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;
            final double diffZ1a = z1 - za;

            final double diffXi1 = xi - x1;
            final double diffYi1 = yi - y1;
            final double diffZi1 = zi - z1;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;
            final double diffZ1a2 = diffZ1a * diffZ1a;

            final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;

            final double rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1) /
                    (Math.log(10.0) * d1a2);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3, 3),
                    new Matrix(3, 3), new Matrix(3, 3));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceFirstOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceSecondOrderNonLinear2D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;

            final double diffXi1 = xi - x1;
            final double diffYi1 = yi - y1;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;

            final double diffXi12 = diffXi1 * diffXi1;
            final double diffYi12 = diffYi1 * diffYi1;

            final double d1a2 = diffX1a2 + diffY1a2;
            final double d1a4 = d1a2 * d1a2;
            final double ln10 = Math.log(10.0);

            final double rssi = fingerprintRssi - 10.0 * pathLossExponent *
                    (diffX1a * diffXi1 + diffY1a * diffYi1) / (ln10 * d1a2) +
                    5.0 * pathLossExponent * ((diffX1a2 - diffY1a2) * (diffXi12 - diffYi12)) / (ln10 * d1a4) +
                    20.0 * pathLossExponent * diffX1a * diffY1a * diffXi1 * diffYi1 / (ln10 * d1a4);

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2, 2),
                    new Matrix(2, 2), new Matrix(2, 2));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceSecondOrderNonLinear3D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;
            final double diffZ1a = z1 - za;

            final double diffXi1 = xi - x1;
            final double diffYi1 = yi - y1;
            final double diffZi1 = zi - z1;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;
            final double diffZ1a2 = diffZ1a * diffZ1a;

            final double diffXi12 = diffXi1 * diffXi1;
            final double diffYi12 = diffYi1 * diffYi1;
            final double diffZi12 = diffZi1 * diffZi1;

            final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
            final double d1a4 = d1a2 * d1a2;
            final double ln10 = Math.log(10.0);

            final double rssi = fingerprintRssi
                    - 10.0 * pathLossExponent * (diffX1a * diffXi1 + diffY1a * diffYi1 + diffZ1a * diffZi1) / (ln10 * d1a2)
                    - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffXi12
                    - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4) * diffYi12
                    - 5.0 * pathLossExponent * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4) * diffZi12
                    + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                    + 20.0 * pathLossExponent * diffY1a * diffZ1a / (ln10 * d1a4) * diffYi1 * diffZi1
                    + 20.0 * pathLossExponent * diffX1a * diffZ1a / (ln10 * d1a4) * diffXi1 * diffZi1;


            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3, 3),
                    new Matrix(3, 3), new Matrix(3, 3));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceThirdOrderNonLinear2D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;

            final double diffXi1 = xi - x1;
            final double diffYi1 = yi - y1;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;

            final double diffXi12 = diffXi1 * diffXi1;
            final double diffYi12 = diffYi1 * diffYi1;

            final double diffXi13 = diffXi12 * diffXi1;
            final double diffYi13 = diffYi12 * diffYi1;

            final double d1a2 = diffX1a2 + diffY1a2;
            final double d1a4 = d1a2 * d1a2;
            final double d1a8 = d1a4 * d1a4;
            final double ln10 = Math.log(10.0);

            final double rssi = fingerprintRssi
                    - 10.0 * pathLossExponent * diffX1a / (ln10 * d1a2) * diffXi1
                    - 10.0 * pathLossExponent * diffY1a / (ln10 * d1a2) * diffYi1
                    - 5.0 * pathLossExponent * (-diffX1a2 + diffY1a2) / (ln10 * d1a4) * diffXi12
                    - 5.0 * pathLossExponent * (diffX1a2 - diffY1a2) / (ln10 * d1a4) * diffYi12
                    + 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4) * diffXi1 * diffYi1
                    - 10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8 * diffXi13
                    - 10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8 * diffYi13
                    - 5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2) * 4.0 * d1a2 * diffY1a) / d1a8 * diffXi12 * diffYi1
                    - 5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2) * 4.0 * d1a2 * diffX1a) / d1a8 * diffXi1 * diffYi12;

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(2, 2),
                    new Matrix(2, 2), new Matrix(2, 2));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear2D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiVarianceThirdOrderNonLinear3D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double fingerprintRssi = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null, null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;
            final double diffZ1a = z1 - za;

            final double diffXi1 = xi - x1;
            final double diffYi1 = yi - y1;
            final double diffZi1 = zi - z1;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;
            final double diffZ1a2 = diffZ1a * diffZ1a;

            final double diffXi12 = diffXi1 * diffXi1;
            final double diffYi12 = diffYi1 * diffYi1;
            final double diffZi12 = diffZi1 * diffZi1;

            final double diffXi13 = diffXi12 * diffXi1;
            final double diffYi13 = diffYi12 * diffYi1;
            final double diffZi13 = diffZi12 * diffZi1;

            final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
            final double d1a4 = d1a2 * d1a2;
            final double d1a8 = d1a4 * d1a4;
            final double ln10 = Math.log(10.0);

            final double value1 = -10.0 * pathLossExponent * diffX1a / (ln10 * d1a2);
            final double value2 = -10.0 * pathLossExponent * diffY1a / (ln10 * d1a2);
            final double value3 = -10.0 * pathLossExponent * diffZ1a / (ln10 * d1a2);
            final double value4 = -5.0 * pathLossExponent * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4);
            final double value5 = -5.0 * pathLossExponent * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4);
            final double value6 = -5.0 * pathLossExponent * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4);
            final double value7 = 20.0 * pathLossExponent * diffX1a * diffY1a / (ln10 * d1a4);
            final double value8 = 20.0 * pathLossExponent * diffY1a * diffZ1a / (ln10 * d1a4);
            final double value9 = 20.0 * pathLossExponent * diffX1a * diffZ1a / (ln10 * d1a4);
            final double value10 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
            final double value11 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
            final double value12 = -10.0 / 6.0 * pathLossExponent / ln10 * (-2.0 * diffZ1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
            final double value13 = -5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
            final double value14 = -5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
            final double value15 = -5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
            final double value16 = -5.0 * pathLossExponent / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
            final double value17 = -5.0 * pathLossExponent / ln10 * (2.0 * diffZ1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
            final double value18 = -5.0 * pathLossExponent / ln10 * (2.0 * diffY1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
            final double value19 = -80.0 * pathLossExponent / ln10 * (diffX1a * diffY1a * diffZ1a * d1a2) / d1a8;

            final double rssi = fingerprintRssi
                    + value1 * diffXi1
                    + value2 * diffYi1
                    + value3 * diffZi1
                    + value4 * diffXi12
                    + value5 * diffYi12
                    + value6 * diffZi12
                    + value7 * diffXi1 * diffYi1
                    + value8 * diffYi1 * diffZi1
                    + value9 * diffXi1 * diffZi1
                    + value10 * diffXi13
                    + value11 * diffYi13
                    + value12 * diffZi13
                    + value13 * diffXi12 * diffYi1
                    + value14 * diffXi12 * diffZi1
                    + value15 * diffXi1 * diffYi12
                    + value16 * diffXi1 * diffZi12
                    + value17 * diffYi12 * diffZi1
                    + value18 * diffYi1 * diffZi12
                    + value19 * diffXi1 * diffYi1 * diffZi1;

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);

            double rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    0.0, new Matrix(3, 3),
                    new Matrix(3, 3), new Matrix(3, 3));

            assertEquals(dist.getMean()[0], rssi, ABSOLUTE_ERROR);
            rssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(rssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiVarianceThirdOrderNonLinear3D(
                    fingerprintRssi, pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null, null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiDifferenceVariance2D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D fingerprintPosition = new InhomogeneousPoint2D(x1, y1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D radioSourcePosition = new InhomogeneousPoint2D(xa, ya);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point2D estimatedPosition = new InhomogeneousPoint2D(xi, yi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiDifferenceVariance2D(
                    pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;

            final double diffXia = xi - xa;
            final double diffYia = yi - ya;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;

            final double diffXia2 = diffXia * diffXia;
            final double diffYia2 = diffYia * diffYia;

            final double d1a2 = diffX1a2 + diffY1a2;
            final double dia2 = diffXia2 + diffYia2;

            final double diffRssi = 5.0 * pathLossExponent * (Math.log10(d1a2) - Math.log10(dia2));

            assertEquals(dist.getMean()[0], diffRssi, ABSOLUTE_ERROR);

            double diffRssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(diffRssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiDifferenceVariance2D(
                    pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    new Matrix(2, 2),
                    new Matrix(2, 2),
                    new Matrix(2, 2));

            assertEquals(dist.getMean()[0], diffRssi, ABSOLUTE_ERROR);
            diffRssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(diffRssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiDifferenceVariance2D(
                    pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance2D(
                    pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance2D(
                    pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null));
        }
    }

    @Test
    public void testPropagateVariancesToRssiDifferenceVariance3D()
            throws IndoorException, AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        for (int t = 0; t < TIMES; t++) {
            final double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            final double x1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double y1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double z1 = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D fingerprintPosition = new InhomogeneousPoint3D(x1, y1, z1);

            final double xa = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double ya = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double za = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D radioSourcePosition = new InhomogeneousPoint3D(xa, ya, za);

            final double xi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double yi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final double zi = randomizer.nextDouble(MIN_POS, MAX_POS);
            final Point3D estimatedPosition = new InhomogeneousPoint3D(xi, yi, zi);

            // test without variance values
            MultivariateNormalDist dist = Utils.propagateVariancesToRssiDifferenceVariance3D(
                    pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, null,
                    null, null,
                    null);

            final double diffX1a = x1 - xa;
            final double diffY1a = y1 - ya;
            final double diffZ1a = z1 - za;

            final double diffXia = xi - xa;
            final double diffYia = yi - ya;
            final double diffZia = zi - za;

            final double diffX1a2 = diffX1a * diffX1a;
            final double diffY1a2 = diffY1a * diffY1a;
            final double diffZ1a2 = diffZ1a * diffZ1a;

            final double diffXia2 = diffXia * diffXia;
            final double diffYia2 = diffYia * diffYia;
            final double diffZia2 = diffZia * diffZia;

            final double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
            final double dia2 = diffXia2 + diffYia2 + diffZia2;

            final double diffRssi = 5.0 * pathLossExponent * (Math.log10(d1a2) - Math.log10(dia2));

            assertEquals(dist.getMean()[0], diffRssi, ABSOLUTE_ERROR);

            double diffRssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(diffRssiVariance, 0.0, ABSOLUTE_ERROR);


            // test with variance values
            dist = Utils.propagateVariancesToRssiDifferenceVariance3D(
                    pathLossExponent, fingerprintPosition,
                    radioSourcePosition, estimatedPosition, 0.0,
                    new Matrix(3, 3),
                    new Matrix(3, 3),
                    new Matrix(3, 3));

            assertEquals(dist.getMean()[0], diffRssi, ABSOLUTE_ERROR);
            diffRssiVariance = dist.getCovariance().
                    getElementAt(0, 0);
            assertEquals(diffRssiVariance, 0.0, ABSOLUTE_ERROR);


            assertNull(Utils.propagateVariancesToRssiDifferenceVariance3D(
                    pathLossExponent, null, radioSourcePosition,
                    estimatedPosition, null,
                    null, null,
                    null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance3D(
                    pathLossExponent, fingerprintPosition, null,
                    estimatedPosition, null, null,
                    null, null));
            assertNull(Utils.propagateVariancesToRssiDifferenceVariance3D(
                    pathLossExponent, fingerprintPosition, radioSourcePosition,
                    null, null,
                    null, null,
                    null));
        }
    }

    private double receivedPower(
            final double equivalentTransmittedPower, final double distance, final double pathLossExponent) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        final double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY), pathLossExponent);
        return equivalentTransmittedPower * k /
                Math.pow(distance, pathLossExponent);
    }
}
