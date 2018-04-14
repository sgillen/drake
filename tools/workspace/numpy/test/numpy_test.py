import subprocess
subprocess.Popen(
    "export -p | sed 's# PWD=# OLD_PWD=#g' > /tmp/env.sh",
    shell=True)

import numpy as np

print(np.get_include())
