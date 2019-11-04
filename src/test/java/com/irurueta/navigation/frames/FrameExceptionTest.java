/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
