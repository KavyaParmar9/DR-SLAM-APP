#!/usr/bin/env sh

#
# Copyright ...
# (standard gradlew script)
#

set -e

APP_BASE_NAME=${0##*/}
APP_HOME=$(dirname "$0")

while [ -h "$0" ] ; do
  ls=$(ls -ld "$0")
  link=$(expr "$ls" : '.*-> \(.*\)$')
  if expr "$link" : '/.*' > /dev/null; then
    APP_HOME=$link
  else
    APP_HOME=$(dirname "$0")/"$link"
  fi
done

CLASSPATH=$APP_HOME/gradle/wrapper/gradle-wrapper.jar
JAVA_EXE=java

exec "$JAVA_EXE" -Xmx64m -Xms64m -classpath "$CLASSPATH" org.gradle.wrapper.GradleWrapperMain "$@"

