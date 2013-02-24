# XBee Enabled Arduino Multimeter Front End
# by Nirav Patel and Donald Cober
# 2009 http://eclecti.cc

from __future__ import with_statement
import serial
import time
import threading
import numpy
import matplotlib.pyplot as pyplot

# we're going to be sharing objects between two threads
lock = threading.Lock()

# each of these is a data line we will be following, voltage, amperage, etc
class DataLogger():
  def __init__(self,timeout=5, subplot=311, name = ''):
    self.data = []
    self.timearray = []
    self.timeout = timeout
    self.subplot = subplot
    self.name = name
    self.scale = 0
    # go to our subplot and set the y axis label
    pyplot.subplot(self.subplot)
    pyplot.ylabel(self.name)
    
  # drop stuff older than can be graphed for time/memory savings
  def dropold(self,now):
    while len(self.timearray) > 0 and (self.timearray[0] < (now - self.timeout)):
      self.timearray.pop(0)
      self.data.pop(0)
      
  # add a new data point to the graph
  def addnew(self,now,data,scale):
    # this is specific to our hardware, when we change the autoranging to
    # another level, clear the plot so the y axis can re-range
    if scale != self.scale:
      pyplot.subplot(self.subplot)
      pyplot.cla()
      pyplot.ylabel(self.name)
    self.scale = scale
    self.data.append(data)
    self.timearray.append(now)
    
  # plot the graph of the last whatever seconds
  def plot(self,now):
    pyplot.subplot(self.subplot)
    pyplot.plot(self.timearray,self.data,'b-', aa=False)
    pyplot.xlim(now-self.timeout,now)
  
  def relabel(self):
    pyplot.subplot(self.subplot)
    pyplot.ylabel(self.name)

# this thread is to interface the XBee
class Arduino(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    # open the serial port.  we could probaly run it faster than this
    self.port = serial.Serial('/dev/ttyUSB0',19200,timeout=1)
    self.dq = []
    self.tq = []
  def run(self):
    while True:
      # read two byte chunks at a time
      s = self.port.read(2)
      if len(s) > 1:
        with lock:
          # once we have the lock, save the bytes and the timestamp
          self.dq.append(s[0])
          self.dq.append(s[1])
          self.tq.append(time.time())

#
class Multimeter():
  def __init__(self):
    # displaying the last 5 seconds of data
    self.timer = 5.0
    pyplot.plot()
    pyplot.show()
    # these are the data lines we are watching
    self.volt = DataLogger(self.timer,311,'Voltage')
    self.amp = DataLogger(self.timer,312,'Amperage')
    self.ohm = DataLogger(self.timer,313,'Resistance')
    self.ard = Arduino()
    self.ard.start()

  def recieve(self):
    with lock:
      # while there is at least one data point left
      while len(self.ard.dq) > 1:
        # get the first byte
        head = ord(self.ard.dq.pop(0))
        # make sure it actually is a header byte
        while len(self.ard.dq)> 1 and not (head & 0x80):
          head = ord(self.ard.dq.pop(0))
        # same for the tail
        # FIXME: this is dirty and will break badly when watching multiple lines
        # We are dropping bytes or going out of order somewhere.
        tail = ord(self.ard.dq.pop(0))
        while len(self.ard.dq)> 1 and (tail & 0x80):
          tail = ord(self.ard.dq.pop(0))
        # get the timestamp too
        now = self.ard.tq.pop(0)
        # parse the data point
        ptype = (head >> 5) & 3
        scale = (head >> 3) & 3
        data = tail + ((head & 7) << 7)
        # first filter on packet type, the only implemented one is voltage now
        if ptype == 0:
          # pick a formula depending on our voltage divider
          if scale == 0:
            data = -0.0207*data+6.7161
          elif scale == 1:
            data = -0.03073*data+11.27016
          elif scale == 2:
            data = -0.056544*data+23.70265
          self.volt.addnew(now,data,scale)
        elif ptype == 1:
          self.amp.addnew(now,data,0)
        elif ptype == 2:
          self.ohm.addnew(now,data,0)
      while len(self.ard.dq) > 0:
        self.ard.dq.pop(0)
      while len(self.ard.tq) > 0:
        self.ard.tq.pop(0)
  
  def update(self, now):
    self.volt.dropold(now)
    self.amp.dropold(now)
    self.ohm.dropold(now)
    self.volt.plot(now)
    self.amp.plot(now)
    self.ohm.plot(now)
    
  def main(self):
    # turn off interactive updating for a massive speed boost
    pyplot.ioff()
    last = time.time()
    while 1:
      now = time.time()
      self.recieve()
      self.update(now)
      # draw it manually once a timestep
      pyplot.draw()
      # clear every five seconds
      if now - last > 5.0:
        pyplot.clf()
        self.volt.relabel()
        self.amp.relabel()
        self.ohm.relabel()

if __name__ == '__main__':
    Multimeter().main()
