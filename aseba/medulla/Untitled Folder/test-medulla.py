#!/usr/bin/python
# -*- coding: utf-8 -*-

import dbus
import dbus.mainloop.glib
import gobject
from optparse import OptionParser
laserop={}
histdisttemp={}
histangletemp={}
hist={},{}
calbsensordist0=-5
calbsensordist1=-5
calbsensordist2=-5
calbsensordist3=0
calbsensorangle0=0
calbsensorangle1=-90
calbsensorangle2=-180
calbsensorangle3=90
newline=""
k=0
histcount=0
#in the simulation
#	front	         
#	  0
#left 1		3 right
#	  2
#	back
#      battery
def get_variables_reply(r):
	print 'variables:'
	print str(r)
	
def get_variables_error(e):
	print 'error:'
	print str(e)
	loop.quit()
def route(ip,op):
	op[2]=ip[0]+calbsensordist0
	op[0]=ip[1]+calbsensordist1
	op[1]=ip[2]+calbsensordist2
	op[3]=ip[3]+calbsensordist3
	if (ip[0]<0):
		op[2]=ip[0]
	if (ip[1]<0):
		op[0]=ip[1]
	if (ip[2]<0):
		op[1]=ip[2]
	if (ip[3]<0):
		op[3]=ip[3]


if __name__ == '__main__':
	parser = OptionParser()
	parser.add_option("-s", "--system", action="store_true", dest="system", default=False,
	help="use the system bus instead of the session bus")
	file = open("datafile.txt", "w")

	(options, args) = parser.parse_args()

	dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

	if options.system:
		bus = dbus.SystemBus()
	else:
		bus = dbus.SessionBus()

	network = dbus.Interface(bus.get_object('ch.epfl.mobots.Aseba', '/'), dbus_interface='ch.epfl.mobots.AsebaNetwork')
	print network.GetNodesList()
	network.LoadScripts("aa.aesl")
	present_dspics = network.GetNodesList()
	network.SendEventName("start_robot",[1])
	network.SetVariable("treel-left", "motor.pid.target_speed", [0])
	#network.SetVariable("treel-right", "rightSpeed", [-10])
	network.SetVariable("treel-right", "motor.pid.target_speed", [-0])
	network.SendEvent(5, [2])
	angle=network.GetVariable("sensor-turret", "sharp.angle")	
	
			
				
	print 'starting loop'
	loop = gobject.MainLoop()
 
	loop.run()
