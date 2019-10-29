package com.irurueta.navigation.inertial;

import org.junit.Test;

import static org.junit.Assert.assertNotNull;

public class InertialExceptionTest {

    @Test
    public void testConstructor() {
        InertialException ex = new InertialException();
        assertNotNull(ex);

        ex = new InertialException("message");
        assertNotNull(ex);

        ex = new InertialException(new Exception());
        assertNotNull(ex);

        ex = new InertialException("message", new Exception());
        assertNotNull(ex);
    }
}
