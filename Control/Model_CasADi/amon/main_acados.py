"""
######################################
    _   __  __  ___  _  _ 
   /_\ |  \/  |/ _ \| \| |
  / _ \| |\/| | (_) | .` |
 /_/ \_\_|  |_|\___/|_|\_|
                          
###############################################
Author: Tilen T.
Date: 2026.06.10
Description: Glavni file - acados verzija
"""

import sys
from pathlib import Path

AMON_ROOT = Path(__file__).resolve().parent
if str(AMON_ROOT) not in sys.path:
    sys.path.insert(0, str(AMON_ROOT))

from model_casadi.control.sim_acados_nmpc import main


if __name__ == "__main__":
    main()
