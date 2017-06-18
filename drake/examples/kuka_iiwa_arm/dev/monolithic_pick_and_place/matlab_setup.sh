#!/bin/bash

# From: bot-spy

for d in . .. ../.. ../../.. ../../../.. "/home/eacousineau/proj/tri/repo/drake-distro/build/install"; do
  if [ -d $d/share/java ]; then
    jd=$d/share/java
    echo Checking $jd
    for f in $jd/lcmtypes_*.jar $jd/lcmspy_plugins_*.jar; do
      if [ -e $f ]; then
        echo "   Found $f"
        CLASSPATH="${CLASSPATH:+$CLASSPATH:}$f"
      fi
    done
  fi
done
echo $CLASSPATH
export CLASSPATH
