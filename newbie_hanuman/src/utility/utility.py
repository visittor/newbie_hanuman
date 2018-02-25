#!/usr/bin/env python

import sys
import imp
import os


def load_module(fileName):
	# open file
	fileObj = file( fileName )

	# create new module
	moduleName = '.'.join( os.path.abspath( fileName ).split( '.' )[:-1] )
	newModule = imp.new_module( moduleName )
	
	# execute fileData in this environment
	oldSysPath = sys.path
	sys.path = [ os.path.dirname( os.path.abspath(fileName) ) ] + sys.path
	exec fileObj in newModule.__dict__
	sys.path = oldSysPath
	
	#	return new module
	return newModule