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

import java.lang.ref.SoftReference;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Contains build data of this library.
 */
public class BuildInfo {

    /**
     * This class logger.
     */
    private static final Logger LOGGER = Logger.getLogger(BuildInfo.class.getName());

    /**
     * Properties file that contains build data.
     * Build data is stored in this file, which is modified each time that compilation is
     * run in the CI server.
     */
    private static final String BUILD_INFO_PROPERTIES = "build-info.properties";

    /**
     * Key to obtain build timestamp from properties file.
     */
    private static final String BUILD_TIMESTAMP_KEY = "BUILD_TIMESTAMP";

    /**
     * Key to obtain groupID of this library from properties file.
     */
    private static final String GROUP_ID_KEY = "GROUP_ID";

    /**
     * Key to obtain artifactId of this library from properties file.
     */
    private static final String ARTIFACT_ID_KEY = "ARTIFACT_ID";

    /**
     * Key to obtain version of this library from properties file.
     */
    private static final String VERSION_KEY = "VERSION";

    /**
     * Key to obtain build number from properties file.
     */
    private static final String BUILD_NUMBER_KEY = "BUILD_NUMBER";

    /**
     * Key to obtain build commit from properties file.
     */
    private static final String COMMIT_KEY = "COMMIT";

    /**
     * Key to obtain build branch from properties file.
     */
    private static final String BRANCH_KEY = "BRANCH";

    /**
     * Format for build timestamp.
     */
    private static final String TIMESTAMP_FORMAT = "yy-MM-dd HH:mm:ss";

    /**
     * Singleton stored in a soft reference (to keep it cached in memory unless
     * memory is claimed).
     */
    private static SoftReference<BuildInfo> reference;

    /**
     * Build timestamp.
     */
    private Date buildTimestamp;

    /**
     * GroupId of this library.
     */
    private String groupId;

    /**
     * ArtifactId of this library.
     */
    private String artifactId;

    /**
     * Version of this library.
     */
    private String version;

    /**
     * Build number.
     */
    private String buildNumber;

    /**
     * Build commit.
     */
    private String commit;

    /**
     * Build branch.
     */
    private String branch;

    /**
     * Constructor.
     */
    private BuildInfo() {
        // loads properties file data.
        try (final var stream = BuildInfo.class.getResourceAsStream(BUILD_INFO_PROPERTIES)) {
            final var props = new Properties();
            props.load(stream);

            final var buildTimestampString = props.getProperty(BUILD_TIMESTAMP_KEY);
            final var format = new SimpleDateFormat(TIMESTAMP_FORMAT, Locale.ENGLISH);
            buildTimestamp = format.parse(buildTimestampString);

            groupId = props.getProperty(GROUP_ID_KEY);
            artifactId = props.getProperty(ARTIFACT_ID_KEY);
            version = props.getProperty(VERSION_KEY);
            buildNumber = props.getProperty(BUILD_NUMBER_KEY);
            commit = props.getProperty(COMMIT_KEY);
            branch = props.getProperty(BRANCH_KEY);
        } catch (final Exception e) {
            LOGGER.log(Level.WARNING, "Failed to load build info", e);
        }
    }

    /**
     * Obtains singleton instance.
     *
     * @return singleton instance.
     */
    public static synchronized BuildInfo getInstance() {
        BuildInfo info;
        if (reference == null || (info = reference.get()) == null) {
            info = new BuildInfo();
            reference = new SoftReference<>(info);
        }

        return info;
    }

    /**
     * Obtains build timestamp.
     *
     * @return build timestamp.
     */
    public Date getBuildTimestamp() {
        return (Date) buildTimestamp.clone();
    }

    /**
     * Obtains groupId of this library.
     *
     * @return groupId of this library.
     */
    public String getGroupId() {
        return groupId;
    }

    /**
     * Obtains artifactId of this library.
     *
     * @return artifactId of this library.
     */
    public String getArtifactId() {
        return artifactId;
    }

    /**
     * Obtains version of this library.
     *
     * @return version of this library.
     */
    public String getVersion() {
        return version;
    }

    /**
     * Obtains build number.
     *
     * @return build number.
     */
    public String getBuildNumber() {
        return buildNumber;
    }

    /**
     * Obtains build commit.
     *
     * @return build commit.
     */
    public String getCommit() {
        return commit;
    }

    /**
     * Obtains build branch.
     *
     * @return build branch.
     */
    public String getBranch() {
        return branch;
    }
}
