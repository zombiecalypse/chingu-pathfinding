#!/bin/sh

operf ruby $1 2>&1 > /dev/null
opreport -lt1
