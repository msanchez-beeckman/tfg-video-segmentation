import os
import sys

if len(sys.argv) < 7:
    print("Usage: python list_flows.py directory format initFrame frameLimit outputFile reverse")
    sys.exit()

directory = sys.argv[1]
flowFormat = sys.argv[2]
initFrame = int(sys.argv[3])
frameLimit = int(sys.argv[4])
outputFile = sys.argv[5]
bwdFlow = sys.argv[6] == 'True'

usingFlo = flowFormat == 'flo'
with open(outputFile, 'w') as outfile:
    for i in range(initFrame, initFrame + frameLimit):
        j = (i + 1) if not bwdFlow else (i - 1)
        if usingFlo:
            if not os.path.exists(os.path.join(directory, '{:05d}to{:05d}.{}'.format(i, j, flowFormat))): continue
            line = directory + '{:05d}to{:05d}.{}'.format(i, j, flowFormat)
            outfile.write(line + '\n')
        else:
            if not os.path.exists(os.path.join(directory, '{:05d}to{:05d}u.{}'.format(i, j, flowFormat))): continue
            if not os.path.exists(os.path.join(directory, '{:05d}to{:05d}v.{}'.format(i, j, flowFormat))): continue
            line1 = directory + '{:05d}to{:05d}u.{}'.format(i, j, flowFormat)
            line2 = directory + '{:05d}to{:05d}v.{}'.format(i, j, flowFormat)
            outfile.write(line1 + '\n')
            outfile.write(line2 + '\n')