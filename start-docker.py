#!/usr/bin/env python3

import sys
sys.path.insert(0, './scripts')
import rundocker
import initialsetup

launch_opt = {
    "run": rundocker.runDocker(),
    "init": initialsetup.main()
}

def main():
    """
    if(launch_opt[sys.argv[1]] is None):
        print('Launch option', sys.argv[1], 'is unknown.')
    else:
        launch_opt[sys.argv[1]]()
    """
    pass

if __name__=="__main__":
    main()