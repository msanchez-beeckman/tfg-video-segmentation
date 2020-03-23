import os
import sys

if len(sys.argv) < 6:
    print("Usage: python list_images.py directory format frameLimit outputFile appendFrameNumber")
    sys.exit()

imageDirectory = sys.argv[1]
imageFormat = sys.argv[2]
frameLimit = int(sys.argv[3])
outputFile = sys.argv[4]
appendFrameNumber = sys.argv[5] == 'True'

with open(outputFile, 'w') as outfile:
    for i in range(frameLimit):
        if not os.path.exists(os.path.join(imageDirectory, '{:05d}.{}'.format(i, imageFormat))): continue
        line = imageDirectory + '{:05d}.{}'.format(i, imageFormat)
        if appendFrameNumber: line = line + ' ' + str(i)
        outfile.write(line + '\n')