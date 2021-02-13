package com.irurueta.navigation.inertial.calibration.magnetometer;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class MagnetometerCalibratorMeasurementTypeTest {

    @Test
    public void testValues() {
        assertEquals(3, MagnetometerCalibratorMeasurementType.values().length);
    }
}
