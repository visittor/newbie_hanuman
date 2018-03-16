#!/usr/bin/env python
import sys

FILE_NAME = "test.log"
class Logger(object):

	def __init__(self):
		self.terminal = sys.stdout
		# self.logFile = open(FILE_NAME, "a")

	def write(self, string):
		self.terminal.write(string)
		with open(FILE_NAME, "a") as f:
			f.write(string)

	def flush(self):
		pass

LOGGER = Logger()

def revert():
	sys.stdout = LOGGER.terminal

def makeLog():
	sys.stdout = LOGGER

makeLog()