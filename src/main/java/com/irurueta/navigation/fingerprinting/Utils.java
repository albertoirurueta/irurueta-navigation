package com.irurueta.navigation.fingerprinting;

public class Utils {

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
}
