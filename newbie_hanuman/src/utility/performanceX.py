#!/usr/bin/env python

from timeit import default_timer as timer

import time

class StopWatch(object):
	def __init__(self):
		self.__n = 0
		self.__elapseTime = None
		self.__avgTime = 0
		self.__startTime = None
		self.__stopTime = None

	def start(self):
		self.__startTime = timer()

	def stop(self):
		self.__stopTime = timer()

		assert self.__startTime is not None, "call stop() before start()"

		self.__elapseTime = self.__stopTime - self.__startTime

		self.__avgTime = (self.__avgTime*self.__n + self.__elapseTime)/(self.__n+1)

		self.__n += 1

		if self.__n > 65535:
			self.resetAvg()

	def resetAvg(self):
		self.__avgTime = 0
		self.__n = 0

	def getAvgTime(self):
		return self.__avgTime

	def getElapseTime(self):
		return self.__elapseTime

	def getN(self):
		return self.__n

	def __enter__(self):
		self.start()

	def __exit__(self, type, value, tb):
		self.stop()

class Timer(object):
	def __init__(self):
		self.__future = None

	def start(self, duration):
		self.__future = time.time() + duration

	def is_timeUp(self):
		# assert self.__future is not None, "call is_timeUp() before start()."
		if self.__future is None:
			return True
		if time.time() > self.__future:
			return True
		return False

	def reset(self):
		self.__future = None