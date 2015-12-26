  Arduino I2C library
  
  Copyright (c) 2011-2012 Wayne Truchsess for Rev 1.0 - Rev 5.0.  All right reserved.
  
  Copyright (c) 2015-2016 Charles Dorval (a.k.a Deskwizard) for Rev 6.0+.  All right reserved.
  
  Download at: https://github.com/deskwizard/Arduino_Library_I2C
  
  
  Rev 6.0 - December 26th, 2015-2016
		  - Modified setSpeed() to be able to pass directly the requested 
			bus speed (Valid speeds: 50/100/200/250/400/500/800 kHz)
		  - Modified scan() printout, added bus speed print, added scan time, changed timeout to 10ms
		  - Added detected(uint8_t address): Returns 1 if chip is detected 
		    at _address, 1 if not detected
		  - Added "Scanner" and "PCF8574" demo sketches to demonstrate library use
		  
  Rev 5.0 - January 24th, 2012
          - Removed the use of interrupts completely from the library
            so TWI state changes are now polled. 
          - Added calls to lockup() function in most functions 
            to combat arbitration problems 
          - Fixed scan() procedure which left timeouts enabled 
            and set to 80msec after exiting procedure
          - Changed scan() address range back to 0 - 0x7F
          - Removed all Wire legacy functions from library
          - A big thanks to Richard Baldwin for all the testing
            and feedback with debugging bus lockups!
			
  Rev 4.0 - January 14th, 2012
          - Updated to make compatible with 8MHz clock frequency
		  
  Rev 3.0 - January 9th, 2012
          - Modified library to be compatible with Arduino 1.0
          - Changed argument type from boolean to uint8_t in pullUp(), 
            setSpeed() and receiveByte() functions for 1.0 compatibility
          - Modified return values for timeout feature to report
            back where in the transmission the timeout occured.
          - added function scan() to perform a bus scan to find devices
            attached to the I2C bus.  Similar to work done by Todbot
            and Nick Gammon
			
  Rev 2.0 - September 19th, 2011
          - Added support for timeout function to prevent 
            and recover from bus lockup (thanks to PaulS
            and CrossRoads on the Arduino forum)
          - Changed return type for stop() from void to
            uint8_t to handle timeOut function 
			
  Rev 1.0 - August 8th, 2011
  
  This is a modified version of the Arduino Wire/TWI 
  library.  Functions were rewritten to provide more functionality
  and also the use of Repeated Start.  Some I2C devices will not
  function correctly without the use of a Repeated Start.  The 
  initial version of this library only supports the Master.


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA