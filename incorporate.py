import re
import sys

# Read in top_block, insert the contents of the radio receiver program in the right place, and add zmq_consumer call
topblock = open(sys.argv[1])
radio = open(sys.argv[2])
outfile = open(sys.argv[3], 'w')
for top_block_line in topblock:
    if re.search('class top_block', top_block_line):
        for radio_line in radio:
            outfile.write(radio_line)
        outfile.write("\n\n")
    if re.search('tb.show()', top_block_line):
        outfile.write("    #tb.show()\n\n")
        outfile.write("    zmq_consumer()\n")
    else:
        outfile.write(top_block_line)
topblock.close()
radio.close()
outfile.close()
print "Finished writing file"
