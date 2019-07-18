#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Top Block
# Generated: Tue Jul 16 00:09:34 2019
##################################################

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt5 import Qt
from PyQt5 import Qt, QtCore
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import zeromq
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import osmosdr
import sip
import sys
import time
from gnuradio import qtgui


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

    amplitude_ring = deque('',80000)  # Buffer that contains the phases of the received samples (1 more than we need)
    
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
        
        # Send to python when ready
        if packet_finished:
            packet = list(amplitude_ring)
            del packet[packet_samples:]
            decode(packet)
            amplitude_ring.clear()
            packet_started = False
            packet_finished = False
            below_thresh = 0


class top_block(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Top Block")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Top Block")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "top_block")

        if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
            self.restoreGeometry(self.settings.value("geometry").toByteArray())
        else:
            self.restoreGeometry(self.settings.value("geometry", type=QtCore.QByteArray))

        ##################################################
        # Variables
        ##################################################
        self.channel_freq = channel_freq = 915003300
        self.samp_rate = samp_rate = 250000
        self.fftsize = fftsize = 512
        self.channel_width = channel_width = 20000
        self.center_freq = center_freq = channel_freq - 50000

        ##################################################
        # Blocks
        ##################################################
        self._channel_width_range = Range(500, 50000, 100, 20000, 200)
        self._channel_width_win = RangeWidget(self._channel_width_range, self.set_channel_width, "channel_width", "counter_slider", float)
        self.top_layout.addWidget(self._channel_width_win)
        self._channel_freq_range = Range(914e6, 916e6, 1000, 915003300, 200)
        self._channel_freq_win = RangeWidget(self._channel_freq_range, self.set_channel_freq, "channel_freq", "counter_slider", float)
        self.top_layout.addWidget(self._channel_freq_win)
        self.zeromq_push_sink_0_0 = zeromq.push_sink(gr.sizeof_float, 1, 'tcp://127.0.0.1:5558', 100, False, -1)
        self.rtlsdr_source_0 = osmosdr.source( args="numchan=" + str(1) + " " + '' )
        self.rtlsdr_source_0.set_sample_rate(samp_rate)
        self.rtlsdr_source_0.set_center_freq(center_freq, 0)
        self.rtlsdr_source_0.set_freq_corr(0, 0)
        self.rtlsdr_source_0.set_dc_offset_mode(0, 0)
        self.rtlsdr_source_0.set_iq_balance_mode(0, 0)
        self.rtlsdr_source_0.set_gain_mode(False, 0)
        self.rtlsdr_source_0.set_gain(0, 0)
        self.rtlsdr_source_0.set_if_gain(20, 0)
        self.rtlsdr_source_0.set_bb_gain(20, 0)
        self.rtlsdr_source_0.set_antenna('', 0)
        self.rtlsdr_source_0.set_bandwidth(0, 0)

        self.qtgui_waterfall_sink_x_0_1 = qtgui.waterfall_sink_c(
        	fftsize, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	center_freq, #fc
        	samp_rate, #bw
        	"Post-filter", #name
                1 #number of inputs
        )
        self.qtgui_waterfall_sink_x_0_1.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0_1.enable_grid(False)
        self.qtgui_waterfall_sink_x_0_1.enable_axis_labels(True)

        if not True:
          self.qtgui_waterfall_sink_x_0_1.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_waterfall_sink_x_0_1.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0_1.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0_1.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0_1.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0_1.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_1_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0_1.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_waterfall_sink_x_0_1_win)
        self.qtgui_waterfall_sink_x_0 = qtgui.waterfall_sink_c(
        	fftsize, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	center_freq, #fc
        	samp_rate, #bw
        	"Raw samples", #name
                1 #number of inputs
        )
        self.qtgui_waterfall_sink_x_0.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0.enable_grid(False)
        self.qtgui_waterfall_sink_x_0.enable_axis_labels(True)

        if not True:
          self.qtgui_waterfall_sink_x_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_waterfall_sink_x_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0.set_intensity_range(-140, 10)

        self._qtgui_waterfall_sink_x_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_waterfall_sink_x_0_win)
        self.qtgui_time_sink_x_0_1_0 = qtgui.time_sink_f(
        	fftsize, #size
        	samp_rate, #samp_rate
        	"Magnitude", #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_0_1_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0_1_0.set_y_axis(0, 0.4)

        self.qtgui_time_sink_x_0_1_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0_1_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0_1_0.set_trigger_mode(qtgui.TRIG_MODE_AUTO, qtgui.TRIG_SLOPE_POS, 0.018, 0, 0, "")
        self.qtgui_time_sink_x_0_1_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0_1_0.enable_grid(False)
        self.qtgui_time_sink_x_0_1_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0_1_0.enable_control_panel(True)
        self.qtgui_time_sink_x_0_1_0.enable_stem_plot(False)

        if not True:
          self.qtgui_time_sink_x_0_1_0.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_0_1_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_0_1_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0_1_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0_1_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0_1_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0_1_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0_1_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_1_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0_1_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_1_0_win)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)
        self.band_pass_filter_0 = filter.fir_filter_ccc(1, firdes.complex_band_pass(
        	1, samp_rate, channel_freq - center_freq - channel_width/2, channel_freq - center_freq + channel_width/2, 1000, firdes.WIN_HAMMING, 6.76))

        ##################################################
        # Connections
        ##################################################
        self.connect((self.band_pass_filter_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.band_pass_filter_0, 0), (self.qtgui_waterfall_sink_x_0_1, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.qtgui_time_sink_x_0_1_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.zeromq_push_sink_0_0, 0))
        self.connect((self.rtlsdr_source_0, 0), (self.band_pass_filter_0, 0))
        self.connect((self.rtlsdr_source_0, 0), (self.qtgui_waterfall_sink_x_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "top_block")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_channel_freq(self):
        return self.channel_freq

    def set_channel_freq(self, channel_freq):
        self.channel_freq = channel_freq
        self.set_center_freq(self.channel_freq - 50000)
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, self.channel_freq - self.center_freq - self.channel_width/2, self.channel_freq - self.center_freq + self.channel_width/2, 1000, firdes.WIN_HAMMING, 6.76))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.rtlsdr_source_0.set_sample_rate(self.samp_rate)
        self.qtgui_waterfall_sink_x_0_1.set_frequency_range(self.center_freq, self.samp_rate)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(self.center_freq, self.samp_rate)
        self.qtgui_time_sink_x_0_1_0.set_samp_rate(self.samp_rate)
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, self.channel_freq - self.center_freq - self.channel_width/2, self.channel_freq - self.center_freq + self.channel_width/2, 1000, firdes.WIN_HAMMING, 6.76))

    def get_fftsize(self):
        return self.fftsize

    def set_fftsize(self, fftsize):
        self.fftsize = fftsize

    def get_channel_width(self):
        return self.channel_width

    def set_channel_width(self, channel_width):
        self.channel_width = channel_width
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, self.channel_freq - self.center_freq - self.channel_width/2, self.channel_freq - self.center_freq + self.channel_width/2, 1000, firdes.WIN_HAMMING, 6.76))

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.rtlsdr_source_0.set_center_freq(self.center_freq, 0)
        self.qtgui_waterfall_sink_x_0_1.set_frequency_range(self.center_freq, self.samp_rate)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(self.center_freq, self.samp_rate)
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, self.channel_freq - self.center_freq - self.channel_width/2, self.channel_freq - self.center_freq + self.channel_width/2, 1000, firdes.WIN_HAMMING, 6.76))


def main(top_block_cls=top_block, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    #tb.show()

    zmq_consumer()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
