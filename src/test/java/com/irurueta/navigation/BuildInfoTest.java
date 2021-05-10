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
package com.irurueta.navigation;

import org.junit.Test;

import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;

public class BuildInfoTest {

    @Test
    public void testGetInstance() {
        final BuildInfo info1 = BuildInfo.getInstance();
        final BuildInfo info2 = BuildInfo.getInstance();

        assertSame(info1, info2);
    }

    @Test
    public void testGetters() {
        final BuildInfo info = BuildInfo.getInstance();

        final String buildNumber = info.getBuildNumber();
        final String commit = info.getCommit();
        final String branch = info.getBranch();

        if (buildNumber != null) {
            Logger.getGlobal().log(Level.INFO, "Build number: {0}", buildNumber);
        }
        if (commit != null) {
            Logger.getGlobal().log(Level.INFO, "Commit: {0}", commit);
        }
        if (branch != null) {
            Logger.getGlobal().log(Level.INFO, "Branch: {0}", branch);
        }

        final Date buildTimestamp = info.getBuildTimestamp();
        final String groupId = info.getGroupId();
        final String artifactId = info.getArtifactId();
        final String version = info.getVersion();

        assertNotNull(buildTimestamp);
        assertNotNull(groupId);
        assertNotNull(artifactId);
        assertNotNull(version);

        Logger.getGlobal().log(Level.INFO, "Build timestamp: {0}",
                buildTimestamp);
        Logger.getGlobal().log(Level.INFO, "Group ID: {0}", groupId);
        Logger.getGlobal().log(Level.INFO, "Artifact ID: {0}", artifactId);
        Logger.getGlobal().log(Level.INFO, "Version: {0}", version);
    }
}
