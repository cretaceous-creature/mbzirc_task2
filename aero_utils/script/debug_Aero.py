#!/usr/bin/env python

from struct import *

import time
import array
import math
import curses
import subprocess
from seed_command import SeedCommand

import sys

class Aero:
	def __init__(self):
		self.POS = 32 * [0]

		self.waist_z = 0

	#Print ServoData
	def Print_POS(self):

                # force torque reading scales. Specific to each ft sensor.
                # Parameters for FT sensor 91155 (Tepura label "A") 
                # obtained from querying sensor through CAN-to-USB adaptor
                # on 2016-12-15
                fx_scale = 1466.0
                fy_scale = 1651.0
                fz_scale = 1994.0
                tx_scale = 31.0
                ty_scale = 33.0
                tz_scale = 21.0

                # Parameters for FT sensor Tepura label "B") 
                # fx_scale = 1417.0
                # fy_scale = 1642.0
                # fz_scale = 1992.0
                # tx_scale = 33.0
                # ty_scale = 35.0
                # tz_scale = 22.0                

                fx = (self.POS[9] - 32768.0)  * fx_scale/32768.0
                fy = (self.POS[10] - 32768.0)  * fy_scale/32768.0
                fz = (self.POS[11] - 32768.0)  * fz_scale/32768.0
                tx = (self.POS[12] - 32768.0)  * tx_scale/32768.0
                ty = (self.POS[13] - 32768.0)  * ty_scale/32768.0
                tz = (self.POS[14] - 32768.0)  * tz_scale/32768.0

		stdscr.addstr(0, 0, "/////////  Position  Data  //////////")
		stdscr.addstr(1, 0, "1:{0}   ".format(self.POS[0]))
		stdscr.addstr(1, 20, "2:{0}   ".format(self.POS[1]))
		stdscr.addstr(1, 40, "3:{0}   ".format(self.POS[2]))
		stdscr.addstr(2, 0, "4:{0}   ".format(self.POS[3]))
		stdscr.addstr(2, 20, "5:{0}   ".format(self.POS[4]))
		stdscr.addstr(2, 40, "6:{0}   ".format(self.POS[5]))
		stdscr.addstr(3, 0, "7:{0}   ".format(self.POS[6]))
		stdscr.addstr(3, 20, "8:{0}   ".format(self.POS[7]))
		stdscr.addstr(3, 40, "9:{0}   ".format(self.POS[8]))
		# stdscr.addstr(4, 0, "10:{0}   ".format(self.POS[9]))
		# stdscr.addstr(4, 20, "11:{0}   ".format(self.POS[10]))
		# stdscr.addstr(4, 40, "12:{0}   ".format(self.POS[11]))
		# stdscr.addstr(5, 0, "13:{0}   ".format(self.POS[12]))
		# stdscr.addstr(5, 20, "14:{0}   ".format(self.POS[13]))
		# stdscr.addstr(5, 40, "15:{0}   ".format(self.POS[14]))
		stdscr.addstr(4, 0, "10:{0}   ".format(fx))
		stdscr.addstr(4, 20, "11:{0}   ".format(fy))
		stdscr.addstr(4, 40, "12:{0}   ".format(fz))
		stdscr.addstr(5, 0, "13:{0}   ".format(tx))
		stdscr.addstr(5, 20, "14:{0}   ".format(ty))
		stdscr.addstr(5, 40, "15:{0}   ".format(tz))

		stdscr.refresh()

	#Get Position Data
	def Read_POS(self,disp=None):
		POS = 31 * [0]

		Rec = seed.AERO_Data_Read(disp)

		if (Rec is None):
			#seed.ser.flushInput()
			return
		else :
			headder = int(Rec[0] + Rec[1] + Rec[2] + Rec[3],16)
			data_cmd = int(Rec[6] + Rec[7],16)

			if (headder == 0xDFFD):
				if (data_cmd == 0x14 or data_cmd == 0x41 or data_cmd == 0x42 or data_cmd == 0x43 or data_cmd == 0x44 or data_cmd == 0x45 or data_cmd == 0x52):
					for i in range(0,31):
						POS[i] = int(Rec[10+i*4] + Rec[10+i*4+1] + Rec[10+i*4+2] + Rec[10+i*4+3],16)
						if (i<9):
							if (POS[i] > 0x7FFF):
								self.POS[i] = POS[i] - pow(2,16)
							elif (POS[i] == 0x7FFF):
								self.POS[i] = 0
							else:
								self.POS[i] = POS[i]
						else:
							self.POS[i] = POS[i]


			else :
				seed.ser.flushInput()

        
        # Convert FT data to N and Nm units using calib parameters
	def convert_FT_readings(self):
                # force torque reading scales. Specific to each ft sensor.
                # Parameters for FT sensor 91155 (Tepura label "A") 
                # obtained from querying sensor through CAN-to-USB adaptor
                # on 2016-12-15
                fx_scale = 1466.0
                fy_scale = 1651.0
                fz_scale = 1994.0
                tx_scale = 31.0
                ty_scale = 33.0
                tz_scale = 21.0

                self.POS[9] = (self.POS[9] - 32768.0)  *fx_scale/32768.0
                self.POS[10] = (self.POS[10] - 32768.0)  *fy_scale/32768.0
                self.POS[11] = (self.POS[11] - 32768.0)  *fz_scale/32768.0
                self.POS[12] = (self.POS[12] - 32768.0)  *tx_scale/32768.0
                self.POS[13] = (self.POS[13] - 32768.0)  *ty_scale/32768.0
                self.POS[14] = (self.POS[14] - 32768.0)  *tz_scale/32768.0
                self.POS[8] = 21112



#----------- main -----------------#
if __name__ == "__main__":

        if len(sys.argv) > 1:
                port = sys.argv[1]
        else:
                port = ("aero_upper")
        print "port %s" % port
        seed=SeedCommand(port,1000000)
	aero=Aero()
	
# Buffer Clear
	seed.ser.flushInput()		#flush input buffer
	seed.ser.flushOutput()		#flush output buffer

# Curses start
	stdscr = curses.initscr()
	curses.noecho()
	curses.cbreak()

	seed.AERO_Snd_Servo_all(0);

	time.sleep(1)


        try:
                while 1:
			seed.AERO_Get_Pos(0)

			aero.Read_POS(0)
			aero.Print_POS()
			time.sleep(0.05)

        finally:
                curses.echo()
                curses.nocbreak()
                curses.endwin()
