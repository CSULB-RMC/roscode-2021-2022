#!/usr/bin/env python3

import os
import re

def getsubmodules(root):
    gm_path = os.path.join(root, ".gitmodules")
    print("Reading submodules in", gm_path, "...")
    sm_file = open(gm_path, "r")
    sm_text = sm_file.read()
    sm = re.findall('(?<=^\[submodule\s").+(?="\])', sm_text, re.MULTILINE)
    sm_file.close()
    print("Detected submodules:")
    for x in sm:
        print(x)
    return sm



def setsubmodulemain(root, sms):
    for x in sms:
        print("Setting module", x, "to main branch...")
        os.system("cd " + os.path.join(root, x) + " && git checkout main && git submodule update --init")
        if os.path.exists(os.path.join(x, ".gitsubmodules")):
            setsubmodulemain(x, getsubmodules(x))


print("Performing initial setup...")
print("Downloading submodules...")
os.system("git submodule update --init")
submoduleslist = getsubmodules(".")
setsubmodulemain(".", submoduleslist)

