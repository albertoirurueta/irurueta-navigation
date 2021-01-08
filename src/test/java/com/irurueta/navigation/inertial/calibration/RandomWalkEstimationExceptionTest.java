package com.irurueta.navigation.inertial.calibration;

import org.junit.Test;

import static org.junit.Assert.assertNotNull;

public class RandomWalkEstimationExceptionTest {

    @Test
    public void testConstructor() {
        RandomWalkEstimationException ex = new RandomWalkEstimationException();
        assertNotNull(ex);

        ex = new RandomWalkEstimationException("message");
        assertNotNull(ex);

        ex = new RandomWalkEstimationException(new Exception());
        assertNotNull(ex);

        ex = new RandomWalkEstimationException("message", new Exception());
        assertNotNull(ex);
    }
}
