import os
import sys

if len(sys.argv) < 7:
    print("Usage: python list_images.py directory format initFrame frameLimit outputFile appendFrameNumber")
    sys.exit()

imageDirectory = sys.argv[1]
imageFormat = sys.argv[2]
initFrame = int(sys.argv[3])
frameLimit = int(sys.argv[4])
outputFile = sys.argv[5]
appendFrameNumber = sys.argv[6] == 'True'

with open(outputFile, 'w') as outfile:
    for i in range(initFrame, initFrame + frameLimit):
        if not os.path.exists(os.path.join(imageDirectory, '{:05d}.{}'.format(i, imageFormat))): continue
        line = imageDirectory + '{:05d}.{}'.format(i, imageFormat)
        if appendFrameNumber: line = line + ' ' + str(i)
        outfile.write(line + '\n')