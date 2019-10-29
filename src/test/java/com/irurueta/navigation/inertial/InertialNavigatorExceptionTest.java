package com.irurueta.navigation.inertial;

import org.junit.Test;

import static org.junit.Assert.assertNotNull;

public class InertialNavigatorExceptionTest {

    @Test
    public void testConstructor() {
        InertialNavigatorException ex = new InertialNavigatorException();
        assertNotNull(ex);

        ex = new InertialNavigatorException("message");
        assertNotNull(ex);

        ex = new InertialNavigatorException(new Exception());
        assertNotNull(ex);

        ex = new InertialNavigatorException("message", new Exception());
        assertNotNull(ex);
    }
}
