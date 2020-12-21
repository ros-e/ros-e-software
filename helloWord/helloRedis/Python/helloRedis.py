#!/usr/bin/env python3

"""
Python3 HelloWorld for Redis Key-Value-Database

Taken from: https://github.com/andymccurdy/redis-py 

Official Redis Commands:
https://redis.io/commands 

If necessary, install Python redis library:
$ pip install redis

"""

import redis    # redis import
import time     # used for examples

""" Example for setting and getting values """
def normaleExample():
  # Connection to local redis db, default port ist 6379
  r = redis.Redis(host="localhost", port=6379, db=0)

  # set value to key
  r.set("helloRedis/python", "test")

  # get value from key
  r.get("helloRedis/python")


""" Example for pipelining commands.
    Pipelines provide support for buffering multiple commands to the server in a single request. 
    They can be used to dramatically increase the performance of groups of commands by 
    reducing the number of back-and-forth TCP packets between the client and server.  """
def pipelineExample():
  # Connection to local redis db, default port ist 6379
  r = redis.Redis(host="localhost", port=6379, db=0)

  r.set("helloRedis/python", "test")

  # Use the pipeline() method to create a pipeline instance
  pipe = r.pipeline()
  # The following SET commands are buffered
  r.set("helloRedis/python/p1", "test1")

  # calls can be chained like:
  r.set("helloRedis/python/p2", "test2").set("helloRedis/python/p3", "test3")

  pipe.get('helloRedis/python')

  # the EXECUTE call sends all buffered commands to the server, returning
  # a list of responses, one for each command.
  pipe.execute()
  # --> [True, True, True, 'test']


"""
  Example for publishing and subscribing to key.
  Redis-Py implements a PubSub object that subscribes to channels and listens for new messages.
"""
def publishSubscribeExample():
  # Connection to local redis db, default port ist 6379
  r = redis.Redis(host="localhost", port=6379, db=0)

  p = r.pubsub()
  # or to ignore subscribe messages
  p = r.pubsub(ignore_subscribe_messages=True) 


  # Subscribe for specific key
  p.subscribe('helloRedis/python', 'helloRedis/python/p1')
  # or subscribe to a pattern
  p.psubscribe('helloRedis/*')

  # >>> p.get_message()
  # {'pattern': None, 'type': 'subscribe', 'channel': 'helloRedis/python', 'data': 1L}
  # >>> p.get_message()
  # {'pattern': None, 'type': 'subscribe', 'channel': 'helloRedis/python/p1', 'data': 2L}
  # >>> p.get_message()
  # {'pattern': None, 'type': 'psubscribe', 'channel': 'helloRedis/*', 'data': 3L}

  # the publish method returns the number matching channel and pattern
  # subscriptions. 'helloRedis/python' matches both the 'helloRedis/python'
  # subscription and the 'helloRedis/*' pattern subscription, so this message will
  # be delivered to 2 channels/patterns
  r.publish('helloRedis/python', 'some data') # == 2

  # >>> p.get_message()
  # {'channel': 'helloRedis/python', 'data': 'some data', 'pattern': None, 'type': 'message'}
  # >>> p.get_message()
  # {'channel': 'helloRedis/python', 'data': 'some data', 'pattern': 'helloRedis/*', 'type': 'pmessage'}

  ### Callback Method ###
  p.subscribe(**{'helloRedis/python': pubSubHandler})
  r.publish('helloRedis/python', 'awesome data')
  p.get_message() # --> HANDLER: awesome data


  ### to get messages ###
  # 1. polling:
  while True:
    message = p.get_message()
    if message:
      # do something with the message
      pass
    time.sleep(0.001) 
  
  # 2. blocking:
  for message in p.listen():
    # do something with the message
    pass


  # CLose publisher subscriber and unsubscribe from all channels
  p.close()  


  



""" Callback Method for PubSub """
def pubSubHandler(message):
  print("HANDLER: ", message['data'])







