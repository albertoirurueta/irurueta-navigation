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

import java.io.IOException;
import java.io.InputStream;
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
     * Location of properties file that contains build data.
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
    private static SoftReference<BuildInfo> sReference;

    /**
     * Build timestamp.
     */
    private Date mBuildTimestamp;

    /**
     * GroupId of this library.
     */
    private String mGroupId;

    /**
     * ArtifactId of this library.
     */
    private String mArtifactId;

    /**
     * Version of this library.
     */
    private String mVersion;

    /**
     * Build number.
     */
    private String mBuildNumber;

    /**
     * Build commit.
     */
    private String mCommit;

    /**
     * Build branch.
     */
    private String mBranch;

    /**
     * Constructor.
     */
    private BuildInfo() {
        //loads properties file data.
        InputStream stream = null;
        try {
            stream = BuildInfo.class.getResourceAsStream(BUILD_INFO_PROPERTIES);

            Properties props = new Properties();
            props.load(stream);

            String buildTimestampString = props.getProperty(
                    BUILD_TIMESTAMP_KEY);
            SimpleDateFormat format = new SimpleDateFormat(TIMESTAMP_FORMAT,
                    Locale.ENGLISH);
            mBuildTimestamp = format.parse(buildTimestampString);

            mGroupId = props.getProperty(GROUP_ID_KEY);
            mArtifactId = props.getProperty(ARTIFACT_ID_KEY);
            mVersion = props.getProperty(VERSION_KEY);
            mBuildNumber = props.getProperty(BUILD_NUMBER_KEY);
            mCommit = props.getProperty(COMMIT_KEY);
            mBranch = props.getProperty(BRANCH_KEY);
        } catch (Exception e) {
            LOGGER.log(Level.WARNING, "Failed to load build info", e);
        } finally {
            if (stream != null) {
                try {
                    stream.close();
                } catch (IOException ignore) {
                    LOGGER.log(Level.WARNING, "Could not properly close stream", ignore);
                }
            }
        }
    }

    /**
     * Obtains singleton instance.
     * @return singleton instance.
     */
    public synchronized static BuildInfo getInstance() {
        BuildInfo info;
        if (sReference == null || (info = sReference.get()) == null) {
            info = new BuildInfo();
            sReference = new SoftReference<>(info);
        }

        return info;
    }

    /**
     * Obtains build timestamp.
     * @return build timestamp.
     */
    public Date getBuildTimestamp() {
        return (Date)mBuildTimestamp.clone();
    }

    /**
     * Obtains groupId of this library.
     * @return groupId of this library.
     */
    public String getGroupId() {
        return mGroupId;
    }

    /**
     * Obtains artifactId of this library.
     * @return artifactId of this library.
     */
    public String getArtifactId() {
        return mArtifactId;
    }

    /**
     * Obtains version of this library.
     * @return version of this library.
     */
    public String getVersion() {
        return mVersion;
    }

    /**
     * Obtains build number.
     * @return build number.
     */
    public String getBuildNumber() {
        return mBuildNumber;
    }

    /**
     * Obtains build commit.
     * @return build commit.
     */
    public String getCommit() {
        return mCommit;
    }

    /**
     * Obtains build branch.
     * @return build branch.
     */
    public String getBranch() {
        return mBranch;
    }
}
