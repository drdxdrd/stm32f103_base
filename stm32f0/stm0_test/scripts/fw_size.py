#!/bin/python3

#import command 
import os
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-s", "--size", dest="size", required=True, help="size util")
parser.add_argument("-n", "--file-name", dest="fname", required=True, help="firmware file name")
parser.add_argument("-f", "--flash-size",dest="max_flash", required=True, help="max flash size")
parser.add_argument("-r", "--ram-size", dest="max_ram", required=True, help="max ram size")

args = parser.parse_args()

cmd = args.size +' '+ args.fname

sz = os.popen(cmd)
sz = sz.read().splitlines()[1].split()

max_flash = int(args.max_flash)
max_ram = int(args.max_ram)
text = int(sz[0] )
data = int(sz[1])
bss = int(sz[2])

flash = text+data
ram = data+bss

flash_used = flash*100/max_flash
ram_used = ram*100/max_ram


print('')
print("flash:\t", flash,'/', max_flash, '\t used', "{0:.1f}%".format(flash_used) ) 
print("ram:\t ", ram,'/', max_ram, '\t used', "{0:.1f}%".format(ram_used) ) 
print('')
