import zmq
import array
from collections import deque
import numpy as np
import math
import matplotlib.pyplot as plt
import numpy as numpy
import scipy.signal

tau = numpy.pi * 2
max_samples = 1000000
minpktlen = 15 * 8   # Minimum number of bits in a packet
minsamples = 10000   # Minumum number of samples expected
invalid_packets = 0
packet_count = 0
debug = True
graph = False

# Whole packet clock recovery adapted from Michael Ossman's wcpr.py
# To learn more, see the following:
# https://www.youtube.com/watch?v=rQkBDMeODHc
# https://github.com/mossmann/clock-recovery/blob/master/wpcr.py

# determine the clock frequency
# input: magnitude spectrum of clock signal (numpy array)
# output: FFT bin number of clock frequency
def find_clock_frequency(spectrum):
    maxima = scipy.signal.argrelextrema(spectrum, numpy.greater_equal)[0]
    while maxima[0] < 2:
        maxima = maxima[1:]
    if maxima.any():
        threshold = max(spectrum[2:-1])*0.8
        indices_above_threshold = numpy.argwhere(spectrum[maxima] > threshold)
        return maxima[indices_above_threshold[0]]
    else:
        return 0

def midpoint(a):
    mean_a = numpy.mean(a)
    mean_a_greater = numpy.ma.masked_greater(a, mean_a)
    high = numpy.ma.median(mean_a_greater)
    mean_a_less_or_equal = numpy.ma.masked_array(a, ~mean_a_greater.mask)
    low = numpy.ma.median(mean_a_less_or_equal)
    return (high + low) / 2

# convert soft symbols into bits (assuming binary symbols)
def slice_bits(symbols):
    symbols_average = numpy.average(symbols)
    if debug:
        print "average amplitude: %s" % symbols_average
    bits = (symbols >= symbols_average)
    return numpy.array(bits, dtype=numpy.uint8)

# Adapted from: https://stackoverflow.com/questions/28370991/converting-bits-to-bytes-in-python#28371638
def getbytes(bits):
    done = False
    while not done:
        byte = 0
        for _ in range(0, 8):
            try:
                bit = next(bits)
            except StopIteration:
                bit = 0
                done = True
            byte = (byte << 1) | bit
        yield byte

def parsepacket(bits):  
    global invalid_packets, packet_count
    packet_count += 1
    bytes = ''
    for b in getbytes(iter(bits)):
        bytes += str(chr(b))
    if len(bits) < minpktlen:
        print "Packet too short (%d bits)" %len(bits)
        invalid_packets += 1
        return
    syncword = (ord(bytes[0])<<8) + ord(bytes[1])
    if(syncword != 0x2dd4):
        print "Invalid packet"
        invalid_packets += 1
        return
    length = ord(bytes[2])
    if(length > len(bytes) - 6):
        print "Invalid length: " %length
        invalid_packets += 1
        return
    crc = (ord(bytes[3+length+1])<<8) + ord(bytes[3+length+2])
    print ":".join("{:02x}".format(ord(bytes[c])) for c in range(4+length+2)),
    print " ",
    for i in range(4+length+2):
        char = bytes[i]
        if ord(char) < 32 or ord(char) >= 127:
            char = '.'
        sys.stdout.write(char)
    print "\n"

def decode(values = []):
    global invalid_packets
    samples = np.array(values)

    # Graph packet
    if graph:
        plt.ion()
        plt.clf()
        plt.show()
        plt.plot(range(np.alen(samples)), samples)
        plt.draw()
        plt.pause(0.000001)
    
    # Clock Recovery
    b = samples > midpoint(samples)
    d = numpy.diff(b)**2
    f = scipy.fft(d, len(samples))
    p = find_clock_frequency(abs(f))
    p = int(p)
    
    # Symbol extraction
    cycles_per_sample = (p*1.0)/len(f)
    clock_phase = 0.5 + numpy.angle(f[p])/(tau)
    if clock_phase <= 0.5:
        clock_phase += 1
    symbols = []
    for i in range(len(samples)):
        if clock_phase >= 1:
            clock_phase -= 1
            symbols.append(samples[i])
        clock_phase += cycles_per_sample
    if debug:
        print("peak frequency index: %d / %d" % (p, len(f)))
        print("samples per symbol: %f" % (1.0/cycles_per_sample))
        print("clock cycles per sample: %f" % (cycles_per_sample))
        print("clock phase in cycles between 1st and 2nd samples: %f" % (clock_phase))
        print("clock phase in cycles at 1st sample: %f" % (clock_phase - cycles_per_sample/2))
        print("symbol count: %d" % (len(symbols)))
        print("invalid packet count: %d / %d" % (invalid_packets,packet_count))

    # Extract bits
    bits=slice_bits(symbols)
    if debug:
        print(list(bits))

    # Align to sync word for beginning of packet
    for i in range(1,50):
        syncword = [0,0,1,0,1,1,0,1,1,1,0,1,0,1,0,0]
        tmpbits = bits[i:]
        if(cmp(syncword,tmpbits[:16].tolist()) == 0):
            bits = bits[i:]
            break

    # Parse and print packet bytes
    parsepacket(list(bits))
    
def zmq_consumer():
    bufsize = 370000            # Number of samples to keep at a time (should be greater than the number of samples in a packet)
    packet_started = False      # Whether or not we've received the first sample of the next/current packet
    packet_finished = False     # Whether or not we have the last sample of the current packet
    packet_samples = 0          # Number of samples in the current packet (once we have them all)
    threshold = 0.025           # Threshold over which we consider a signal to have been detected
    below_thresh = 0            # Number of samples since we've been below the threshold
    trailing_pad = 6000         # How many samples below the thresold do we want to keep at the end of each packet

    context = zmq.Context()
    results_amplitude = context.socket(zmq.PULL)
    results_amplitude.connect("tcp://127.0.0.1:5558")

    amplitude_ring = deque('',80000)  # Buffer that contains the amplitude samples
    
    while True:
        # Read in the amplitude samples
        raw_amplitude = results_amplitude.recv()
        amp_list = array.array('f', raw_amplitude) # struct.unpack will be faster
        if len(amp_list) >= 0:
            for f in amp_list:
                amplitude_ring.append(f)
                if packet_finished:
                    continue
                if not packet_started:
                    if f > threshold:
                        packet_started = True
                        # Remove all the samples but the most recent ones (truncate the deque)
                        for i in range(len(amplitude_ring)-200):
                            amplitude_ring.popleft()
                else:
                    if f < threshold:
                        below_thresh += 1
                        if below_thresh >= trailing_pad:
                            multiplier = int(len(amplitude_ring) / 8192);
                            if len(amplitude_ring) == multiplier * 8192:
                                packet_finished = True
                                packet_samples = len(amplitude_ring)
                    else:
                        below_thresh = 0
        
        # Send to demodulator when ready
        if packet_finished:
            packet = list(amplitude_ring)
            del packet[packet_samples:]
            decode(packet)
            amplitude_ring.clear()
            packet_started = False
            packet_finished = False
            below_thresh = 0
