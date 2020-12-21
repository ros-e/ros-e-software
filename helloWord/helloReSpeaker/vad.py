#!/usr/bin/env python3

from tuning import Tuning
import usb.core
import usb.util
import time

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
#print dev
if dev:
  Mic_tuning = Tuning(dev)
  print(Mic_tuning.is_voice())
  while True:
    try:
      print(Mic_tuning.is_voice())
      time.sleep(0.5)
    except KeyboardInterrupt:
      break
