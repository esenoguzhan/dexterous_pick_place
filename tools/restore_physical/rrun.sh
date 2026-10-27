#!/bin/bash
# rrun.sh SCRIPT.py [args...]: run a tools/restore_physical script inside the restore_main container.
exec docker exec -i -e PYTHONUNBUFFERED=1 restore_main bash -c \
  'source /ws/install/setup.bash; cd /repo/tools/restore_physical; exec python3 "$@"' _ "$@"
