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
minDist=100000
bestdir=0
histcount=0
mostProbDist = 0
mostProbX = 0
mostProbY = 0
mostProbDir =0
minX=-2
minY=2
minX=-2
minY=2
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
	while(k<100):
		angle=network.GetVariable("sensor-turret", "sharp.angle")
		laser=network.GetVariable("sensor-turret", "sharp.dist")
		route(laser,laserop)		
		histdisttemp[k]=laserop[0]   
  		histdisttemp[k+1]=laserop[1]    
		histdisttemp[k+2]=laserop[2]     		
		histdisttemp[k+3]=laserop[3]    
  		if ((angle[0]>=0)and(angle[0]<90)): 
			histangletemp[k]=angle[0]+0
  			histangletemp[k+1]=angle[0]+90
			histangletemp[k+2]=angle[0]-180
			histangletemp[k+3]=angle[0]-90
  		if ((angle[0]>=90)and(angle[0]<180)): 
			histangletemp[k]=angle[0]+0
  			histangletemp[k+1]=angle[0]-270
			histangletemp[k+2]=angle[0]-180
			histangletemp[k+3]=angle[0]-90
  		if ((angle[0]>=180)and(angle[0]<270)): 
			histangletemp[k]=angle[0]-360
  			histangletemp[k+1]=angle[0]-270
			histangletemp[k+2]=angle[0]-180
			histangletemp[k+3]=angle[0]-90
  		if ((angle[0]>=270)and(angle[0]<=360)): 
			histangletemp[k]=angle[0]-360
  			histangletemp[k+1]=angle[0]-270
			histangletemp[k+2]=angle[0]-180
			histangletemp[k+3]=angle[0]-450

		newline=str(histangletemp[k])+","
		#newline="sdf"+str(histdist[k])+","+str(histangle[k])+";"+str(histdist[k+1])+","+str(histangle[k+1])+";"+str(histdist[k+2])+","+str(histangle[k+2])+";"+str(histdist[k+3])+","+str(histangle[k+3])+";"		
		file.write(newline)
		k=k+4
	hist[0][0]=histangletemp[0]
	hist[1][0]=histdisttemp[0]
	for i in xrange(0,k-1):
		findvalue=False
		for subi in xrange(0,histcount+1):
			print abs(histangletemp[i]-hist[0][subi])	
			if (abs(histangletemp[i]-hist[0][subi])<1):
					if (hist[1][subi]<0):
						hist[1][subi]=histdisttemp[i]		
						break
			findvalue=True

		if (findvalue is False):
			histcount+=1
			hist[0][histcount]=histangletemp[i]
			hist[1][histcount]=histdisttemp[i]
	#sort
	temp1=0
	temp2=0
	#for m in xrange(0,histcount):
		#for n in xrange(m,histcount):
			#if(hist[0][n]<=hist[0][m]):
				#temp1=hist[0][m]
				#temp2=hist[1][m]
				#hist[0][m]=hist[0][n]
				#hist[1][m]=hist[1][n]
				#hist[0][n]=temp1
				#hist[1][n]=temp2
	
	fd = open( "final.txt" )
	content = fd.readline()
	k=0
	worldno = 0
	posx=0
	posy=0
	posz=0
	state=-1
	worldhist={},{}
	while (content != "" ):
		content = fd.readline()
		#print content
		words = content.split(",")
			print len(words)
		try:
			worldno=int(words[0])
			posx=float(words[1])
			posy=float(words[2])
			posz=float(words[3])
			state=float(words[4])
		except ValueError:
				worldno =-1

		for word in xrange(5,len(words)-1,2):
			# prints each word on a line
			worldhist[1][word]=float(words[word+1])
			worldhist[0][word]=float(words[word])
			print word	
		k=k+1
		if (state==0):
			for l in xrange(-180,+180,360/histcount):
				dist=0
				for m in xrange(0,histcount):
					for n in xrange(0,len(words)):
						angle=hist[0][m]+l
						if (angle>180):
							angle=angle-360
						if (angle<-180):
							angle=360-angle	
						if (abs(angle-worldhist[0][n])<=1):
							v=hist[1][m]-worldhist[1][n]
							break
						dist+=v*v
				if ((dist<minDist)or(l==-180)):
					minDist = (dist);
					bestDir = l;
			
			if (((minDist<mostProbDist)and(((posx>minX)&&(posy>minY)and(posx<maxX)and(posy<maxY))))or((posx==0)and(posy==0))):
				mostProbDist = minDist
				mostProbX = posx
				mostProbY = posy
				mostProbDir = bestDir
	print 'starting loop'
	loop = gobject.MainLoop()
 
	loop.run()
