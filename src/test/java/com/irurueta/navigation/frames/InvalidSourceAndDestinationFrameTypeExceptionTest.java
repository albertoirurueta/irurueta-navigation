package com.irurueta.navigation.frames;

import org.junit.Test;

import static org.junit.Assert.*;

public class InvalidSourceAndDestinationFrameTypeExceptionTest {

    @Test
    public void testConstructor() {
        InvalidSourceAndDestinationFrameTypeException ex = new InvalidSourceAndDestinationFrameTypeException();
        assertNotNull(ex);

        ex = new InvalidSourceAndDestinationFrameTypeException("message");
        assertNotNull(ex);
        assertEquals(ex.getMessage(), "message");

        final Exception cause = new Exception();
        ex = new InvalidSourceAndDestinationFrameTypeException(cause);
        assertNotNull(ex);
        assertSame(ex.getCause(), cause);

        ex = new InvalidSourceAndDestinationFrameTypeException("message", cause);
        assertNotNull(ex);
        assertEquals(ex.getMessage(), "message");
        assertSame(ex.getCause(), cause);
    }
}
