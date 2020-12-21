#!/usr/bin/env python3

from tuning import Tuning
import usb.core
import usb.util
import redis
import time

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
#print dev
if dev:

  # Connection to local redis db, default port ist 6379
  r = redis.Redis(host="localhost", port=6379, db=0)
  # ignore subscribe messages
  #p = r.pubsub(ignore_subscribe_messages=True) 

  Mic_tuning = Tuning(dev)
  print(Mic_tuning.is_voice())
  while True:
    try:
      if Mic_tuning.is_voice():
        direction = Mic_tuning.direction
        print(direction)

        r.publish("robot:doa", direction)

      time.sleep(0.2)
    except KeyboardInterrupt:
      break
