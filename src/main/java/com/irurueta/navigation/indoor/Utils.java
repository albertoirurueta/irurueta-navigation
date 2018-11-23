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
import com.irurueta.navigation.indoor.radiosource.RssiRadioSourceEstimator;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.StatisticsException;

@SuppressWarnings("Duplicates")
public class Utils {

    /**
     * Speed of light expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = RssiRadioSourceEstimator.SPEED_OF_LIGHT;

    /**
     * Prevents instantiation
     */
    private Utils() { }

    /**
     * Converts from dBm's to linear power value expressed in mW.
     * @param dBm value to be converted expressed in dBm's.
     * @return converted value expressed in mW.
     */
    public static double dBmToPower(double dBm) {
        return Math.pow(10.0, dBm / 10.0);
    }

    /**
     * Converts from mW to logarithmic power value expressed in dBm's.
     * @param mW value to be converted expressed in mW's.
     * @return converted value expressed in dBm's.
     */
    public static double powerTodBm(double mW) {
        return 10.0 * Math.log10(mW);
    }

    /**
     * Propagates variance on received power measure into distance variance by considering the following formula
     * for received power (expressed in dBm's):
     * rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance,
     * where logSqrDistance is the logarithm in base 10 of the squared distance logSqrDistance = Math.log(d^2).
     * Taking into account the previous formula, distance can be expressed as:
     * d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
     * where kdB is a constant having the following expression:
     * kdB = 10.0 * log(c / (4 * pi * f)),
     * where c is the speed of light and f is the frequency.
     * @param txPower transmitted power expressed in dBm's.
     * @param rxPower received power expressed in dBm's.
     * @param pathLossExponent path loss exponent.
     * @param frequency frequency expressed in Hz.
     * @param rxPowerVariance received power variance.
     * @return distance variance.
     */
    public static double propagatePowerVarianceToDistanceVariance(double txPower,
            double rxPower, double pathLossExponent, double frequency,
            Double rxPowerVariance) {
        if (rxPowerVariance == null) {
            return 0.0;
        }

        double k = SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        double kdB = 10.0 * Math.log10(k);

        //distance follows the following expression:
        //d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
        //where kdB is a constant having the following expression:
        //kdB = 10.0 * log(c / (4 * pi * f)),
        //where c is the speed of light and f is the frequency.

        //hence, if the only unknown is the received power (x = rxPower), we can express the distance as:
        //d = f(x) = 10.0^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))

        //if we know the variance of received power var(x), then the variance of the distance will be:
        //var(d) = var(f(x)) = (f'(E(x)))^2*var(x)
        //where f'(x) is the derivative of f(x) evaluated at E(x) = rxPower

        //the derivative f'(x) has the following expression:
        //f'(x) = -ln(10)/(10.0*pathLossExponent)*10^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))

        //evaluate derivative at E(x) = rxPower:
        double tenPathLossExponent = 10.0 * pathLossExponent;
        double derivativeF = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                (pathLossExponent * kdB + txPower - rxPower) / tenPathLossExponent);

        return derivativeF * derivativeF * rxPowerVariance;
    }

    /**
     * Propagates provided variances (transmitted power variance, received power variance and pathloss variance) into
     * distance variance by considering the following formula for received power (expressed in dBm's):
     * rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance,
     * where logSqrDistance is the logarithm in base 10 of the squared distance logSqrDistance = Math.log(d^2).
     * Taking into account the previous formula, distance can be expressed as:
     * d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
     * where kdB is a constant having the following expression:
     * kdB = 10.0 * log(c / (4 * pi * f)),
     * where c is the speed of light and f is the frequency.
     * @param txPower transmitted power expressed in dBm's.
     * @param rxPower received power expressed in dBm's.
     * @param pathLossExponent path loss exponent.
     * @param frequency frequency expressed in Hz.
     * @param txPowerVariance transmitted power variance.
     * @param rxPowerVariance received power variance.
     * @param pathLossExponentVariance path loss exponent variance.
     * @return a normal distribution containing both expected distance and its variance.
     * @throws IndoorException if something fails.
     */
    public static MultivariateNormalDist propagateVariancesToDistanceVariance(final double txPower,
            final double rxPower, final double pathLossExponent, final double frequency,
            Double txPowerVariance, Double rxPowerVariance,
            Double pathLossExponentVariance) throws IndoorException {
        if (txPowerVariance == null && rxPowerVariance == null && pathLossExponentVariance == null) {
            return null;
        }

        double[] mean = new double[]{ txPower, rxPower, pathLossExponent };
        Matrix covariance = Matrix.diagonal(new double[]{
                txPowerVariance != null ? txPowerVariance : 0.0,
                rxPowerVariance != null ? rxPowerVariance : 0.0,
                pathLossExponentVariance != null ? pathLossExponentVariance : 0.0
        });

        try {
            return MultivariateNormalDist.propagate(new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(double[] x, double[] y, Matrix jacobian) {
                    double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
                    double kdB = 10.0 * Math.log10(k);

                    //received power in dBm's follows the equation:
                    //rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance

                    //hence, distance follows the following expression:
                    //d = 10.0^((pathLossExponent * kdB + txPower - rxPower)/(10.0 * pathLossExponent))
                    //where kdB is a constant having the following expression:
                    //kdB = 10.0 * log(c / (4 * pi * f)),
                    //where c is the speed of light and f is the frequency.

                    double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower) / (5.0 * pathLossExponent);

                    //where logSqrDistance = Math.log10(sqrDistance)
                    //and sqrDistance = distance * distance, hence
                    //logSqrDistance = Math.log10(distance * distance) = 2 * Math.log10(distance)

                    y[0] = Math.pow(10.0,logSqrDistance / 2.0);



                    //compute gradient (is a jacobian having 1 row and 3 columns)

                    //derivative of distance respect to transmitted power is:

                    //if the only unknown is the transmitted power (x = txPower), then:
                    //d = f(x) = 10.0^((pathLossExponent * kdB + x - rxPower)/(10.0 * pathLossExponent))
                    //and the derivative is
                    //f'(x) = ln(10)/(10.0 * pathLossExponent)*10.0^((pathLossExponent * kdB + x - rxPower)/(10.0 * pathLossExponent))
                    double tenPathLossExponent = 10.0 * pathLossExponent;
                    double derivativeTxPower = Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                            (pathLossExponent * kdB + txPower - rxPower) / tenPathLossExponent);

                    //derivative of distance respect to received power is:

                    //if the only unknown is the received power (x = rxPower), then:
                    //d = f(x) = 10.0^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))
                    //and the derivative is
                    //f'(x) = -ln(10)/(10.0*pathLossExponent)*10^((pathLossExponent * kdB + txPower - x)/(10.0 * pathLossExponent))
                    double derivativeRxPower = -Math.log(10.0) / tenPathLossExponent * Math.pow(10.0,
                            (pathLossExponent * kdB + txPower - rxPower) / tenPathLossExponent);


                    //derivative respect to path loss exponent is:

                    //if the only unknown is the path loss exponent (x = pathLossExponent), then:
                    //d = f(x) = 10.0^((x * kdB + txPower - rxPower)/(10.0 * x))
                    //and the derivative is:
                    //f'(x) = ln(10) * g'(x) * 10.0^(g(x))
                    //where g(x) is:
                    //g(x) = (x * kdB + txPower - rxPower) / (10.0 * x)
                    //and the derivative of g(x) is:
                    //g'(x) = (kdB * 10.0 * x - 10.0 * (x * kdB + txPower - rxPower)) / (10.0 * x)^2
                    //Hence:
                    //f'(x) = lng(10) * (kdB * 10.0 * x - 10.0 * (x * kdB + txPower - rxPower)) / (10.0 * x)^2 * 10.0^((x * kdB + txPower - rxPower)/(10.0 * x))

                    double g = (pathLossExponent * kdB + txPower - rxPower) / (10.0 * pathLossExponent);
                    double derivativeG = (kdB * 10.0 * pathLossExponent -
                            10.0 * (pathLossExponent * kdB + txPower - rxPower)) /
                            Math.pow(10.0 * pathLossExponent, 2.0);

                    double derivativePathLossExponent = Math.log(10.0) * derivativeG * Math.pow(10.0, g);

                    jacobian.setElementAtIndex(0, derivativeTxPower);
                    jacobian.setElementAtIndex(1, derivativeRxPower);
                    jacobian.setElementAtIndex(2, derivativePathLossExponent);
                }

                @Override
                public int getNumberOfVariables() {
                    return 1;
                }
            }, mean, covariance);
        } catch (AlgebraException | StatisticsException e) {
            throw new IndoorException(e);
        }
    }
}
