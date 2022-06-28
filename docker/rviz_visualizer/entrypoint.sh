#!/bin/bash
set -e

# setup ros environment
source "/home/movel_ws/devel/setup.bash"
exec "$@"