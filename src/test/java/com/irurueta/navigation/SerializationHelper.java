/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

public class SerializationHelper {

    public static <T extends Serializable> byte[] serialize(final T object) throws IOException {
        try (final var byteArrayOutputStream = new ByteArrayOutputStream()) {
            try (final var objectOutputStream = new ObjectOutputStream(byteArrayOutputStream)) {
                objectOutputStream.writeObject(object);
            }

            byteArrayOutputStream.flush();

            return byteArrayOutputStream.toByteArray();
        }
    }

    public static <T extends Serializable> T deserialize(final byte[] bytes) throws IOException,
            ClassNotFoundException {
        try (final var byteArrayInputStream = new ByteArrayInputStream(bytes)) {
            try (final var objectInputStream = new ObjectInputStream(byteArrayInputStream)) {
                //noinspection unchecked
                return (T) objectInputStream.readObject();
            }
        }
    }
}
