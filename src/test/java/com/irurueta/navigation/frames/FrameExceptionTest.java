package com.irurueta.navigation.frames;

import org.junit.Test;

import static org.junit.Assert.*;

public class FrameExceptionTest {

    @Test
    public void testConstructor() {
        FrameException ex = new FrameException();
        assertNotNull(ex);

        ex = new FrameException("message");
        assertNotNull(ex);
        assertEquals(ex.getMessage(), "message");

        final Exception cause = new Exception();
        ex = new FrameException(cause);
        assertNotNull(ex);
        assertSame(ex.getCause(), cause);

        ex = new FrameException("message", cause);
        assertNotNull(ex);
        assertEquals(ex.getMessage(), "message");
        assertSame(ex.getCause(), cause);
    }
}
