#!/usr/bin/env python3

import os
import re

class submodule_info:
    name =""
    path = ""
    url = ""
    branch = ""
    def __str__(self):
        return "Submodule name: " + self.name + "\npath: " + self.path + "\nurl: " + self.url + "\nbranch: " + self.branch

def parseelement(element_string):
    e = submodule_info()
    ret = re.findall('(?<=\").*(?=\")', element_string)
    if ret:
        e.name = ret[0]
    else:
        e.name = ""
    
    ret = re.findall('(?<=\tpath\s\=\s).*(?=\n?)', element_string)
    if ret:
        e.path = ret[0]
    else:
        e.path = ""
    
    ret = re.findall('(?<=\turl\s\=\s).+(?=\n?)', element_string)
    if ret:
        e.url = ret[0]
    else:
        e.url = ""
    
    ret = re.findall('(?<=\tbranch\s\=\s).+(?=\n?)', element_string)
    if ret:
        e.branch = ret[0]
    else:
        e.branch = "main" #probably not always safe to assume main but.. whatever
    print(e.name)
    return e

def parsesubmodules(root):
    gm_path = os.path.join(root, ".gitmodules")
    print("Reading submodules in", gm_path, "...")
    sm_file = open(gm_path, "r")
    sm_text = sm_file.read()
    #sm = re.findall('(?<=^\[submodule\s").+(?="\])', sm_text, re.MULTILINE)
    sm = re.findall('(\[\s*submodule\s*.*\n(\t.*\n?)+)', sm_text)
    sm_file.close()
    print("Detected submodules:")
    sm_elements =[]
    for x in sm:
        sm_elements.append(parseelement(x[0]))
    print("")
    return sm_elements



def setsubmodulemain(root, sms):
    for x in sms:
        print("Setting module", x.name, "to main branch...")
        fp = os.path.join(root, x.path)
        os.system("cd " + fp + " && git checkout " + x.branch+ " && git submodule update --init")
        if os.path.exists(os.path.join(fp, ".gitmodules")):
            setsubmodulemain(fp, parsesubmodules(fp))

def main():
    print("Performing initial setup...")
    print("Downloading submodules...")
    os.system("git submodule update --init")
    submoduleslist = parsesubmodules(".")
    setsubmodulemain(".", submoduleslist)

if __name__=="__main__":
    main()