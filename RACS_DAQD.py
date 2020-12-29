#!/usr/bin/env python3
#  -*- coding: utf-8 -*-

"""
	Description:
		 This code conducts a finite high sample rate data acquisition 
		 scan using an MCC118 DAQ Hat. This program also utilizes a 
		 Ronoth LoStik as a method to initiate the scan using a long-
		 distance LoRa radio signal on the 913 MHz band. The data is 
		 then saved to a CSV file onboard Raspberry Pi.
"""

import sys
import time
import serial
import os
import csv  
import errno
import binascii
import RPi.GPIO as GPIO
import glob
from serial.threaded import LineReader, ReaderThread
from daqhats import mcc118, OptionFlags, TriggerModes, HatIDs, HatError
from daqhats_utils import select_hat_device, enum_mask_to_string, \
    chan_list_to_mask
from datetime import datetime

# Name and number of the DAQ system that this instance of the code is 
# installed. These values are used for file naming and radio response 
# sequencing.
DAQ_NAME = 'DAQ_D'
DAQ_NUM = 4

# Total number of DAQ/RADIO Systems in measurement array (this value is 
# used for timing the radio responses from each DAQ).
NUM_OF_DAQS = 6

# Total length of delay (s) between trigger transmission and initiation  
# of recording... this delay allows time for responses from each DAQ
RESPONSE_DELAY = 18

# MCC118 Channels to record from
channels = [0, 7]

# Desired Sample Rate Per Channel (Hz) - CANNOT exceed 100 kHz combined for MCC118
global scan_rate
scan_rate = 50000.0 

# Desired recording length (seconds)
recording_length = 30

# .csv file location
basepath = '/home/pi/Desktop' 
mypath = basepath + '/' + DAQ_NAME + '/DATA'

# DAQ clearing terminal inputs
CURSOR_BACK_2 = '\x1b[2D'
ERASE_TO_END_OF_LINE = '\x1b[0K'

# Since the read_request_size is set to -1 (READ_ALL_AVAILABLE), 
# the scan will return immediately with whatever samples are available 
# (up to user_buffer_size).
READ_ALL_AVAILABLE = -1

# Pin (GPIO 21) responsible for ending script and shutting down RPi. 
# Shutdown is initiated when pin is driven HIGH by a momentary switch
PWR_PIN = 21

# Pin (GPIO 24) responsible for triggering MCC118 DAQ Hat. Is driven
# HIGH when specific radio signal is received from LoStik
TRIGGER_PIN = 24

# Pin (GPIO 18) used for LED indicating that system is ready to record
# and is waiting for LoStik trigger.
PRIMED_LED = 18

# Pin (GPIO 25) used for LED indicating that system has been triggered 
# and is in the process of collecting data from MCC118 DAQ Hat.
RECORDING_LED = 25

# Pin (GPIO 18) used for LED indicating that recording has completed 
# and is saving CSV file.
COMPLETE_LED = 5

# Flag indicating that radio trigger has been recieved (FLAG = 0) and 
# allows the LoStik to break from its "waiting to receive" loop
CMD_RECEIVED = 1

# Flag indicating that the radio recieved a shutdown command (FLAG = 1)
CMD_SHUTDOWN = 0

# Flag to verify LoStik is Inserted
global LoStikInserted
LoStikInserted = 1

# Message that must be received by LoStik to initiate recording. NOTE:
# only the hexadecimal portion must be sent by transmitter LoStik
REC_SIG = 'radio_rx  4D43435354' # 4D43435354 = "MCCST"
SHUTDOWN_SIG = 'radio_rx  4D43435344' # 4d43435344 = "MCCSD"
PING_SIG = 'radio_rx  4D43435047' # 4D43435047 = "MCCPG"
RECORDINGLENGTH_SIG = 'radio_rx  4D4343524C' # 4D43435047 = "MCCRL "

# Response transmitted by radio when a connection to the LoStik has been
# established. *MUST STAGGER RESPONSES FROM MULTIPLE RADIOS
READY_RESPONSE = DAQ_NAME + ' Rdy'
READY_HEX = binascii.hexlify(READY_RESPONSE.encode()).decode()

# Response transmitted by radio when the trigger message has been
# received. *MUST STAGGER RESPONSES FROM MULTIPLE RADIOS
TRIGG_RESPONSE = DAQ_NAME + ' Trg'
TRIGG_HEX = binascii.hexlify(TRIGG_RESPONSE.encode()).decode()

# Response transmitted by radio when the trigger message has been
# received. **MUST STAGGER RESPONSES FROM MULTIPLE RADIOS
SHUTDOWN_RESPONSE = DAQ_NAME + ' SDn'
SHUTDOWN_HEX = binascii.hexlify(SHUTDOWN_RESPONSE.encode()).decode()



# Functions to control the staggering of radio responses from DAQS to
# avoid "talking over each other"
def RadioResponseFirstDelay():
    Delay_Increment = RESPONSE_DELAY/NUM_OF_DAQS
    global StartTime
    StartTime = (DAQ_NUM-1)*Delay_Increment
    time.sleep(StartTime)
def RadioResponseSecondDelay():
    FinishTime = RESPONSE_DELAY - StartTime
    time.sleep(FinishTime)
    
class PrintLines(LineReader):
    	
    def connection_made(self, transport):
        print("     Connected to LoStik")
        self.transport = transport
        self.send_cmd("sys set pindig GPIO11 1")
        self.send_cmd('mac pause') # Prepare LoStik to receive
        self.send_cmd('radio set pwr 15', delay=1) # Power for transmission. WARNING - possible to exceed FCC allowable limits. Use only to compensate for line losses to antenna.
        #self.send_cmd('radio set wdt 0', delay=1) # Disable watchdog timer for continuous reception

        # Sending staggered response
        RadioResponseFirstDelay()
        self.send_cmd('radio tx '+READY_HEX, delay=0)
        RadioResponseSecondDelay()
        
        self.send_cmd('radio rx 0') # Engages continuous reception mode
        self.send_cmd("sys set pindig GPIO10 0") # Blue LED - LOW	
	   
    def handle_line(self, data):
        if data == "ok" or data == 'busy' or data == 'radio_tx_ok':
            return
        if data == "radio_err":
            self.send_cmd('radio rx 0')
            return
        if data[:10] == 'radio_rx  ':
            try:
                self.send_cmd("sys set pindig GPIO10 1", delay=0) # Blue LED - HIGH            
                print('     '+binascii.unhexlify(data[10:]).decode())
                time.sleep(.1)
                self.send_cmd("sys set pindig GPIO10 0", delay=1) # Blue LED - LOW            
            except:
                print("     Cannot decode message")
                time.sleep(.1)
                self.send_cmd("sys set pindig GPIO10 0", delay=1)
                self.send_cmd('radio rx 0')
        else:
            self.send_cmd("sys set pindig GPIO10 1", delay=0) # Blue LED - HIGH
            print('     '+data) # Print data received (with formatting spaces)
            time.sleep(.1)
            self.send_cmd("sys set pindig GPIO10 0", delay=1) # Blue LED - LOW

		
        global CMD_RECEIVED # Define trigger flag as global variable
        global CMD_SHUTDOWN # Define shutdown flag as global variable
        global recording_length
        
        # Handle a trigger message        
        if data == REC_SIG: # Trigger Message
			# Turning off red LED
            self.send_cmd("sys set pindig GPIO11 0")

            # Sending staggered response
            RadioResponseFirstDelay()
            self.send_cmd('radio tx '+TRIGG_HEX, delay=0)
            RadioResponseSecondDelay()
            
            # Setting trigger flag
            CMD_RECEIVED = 0

        # Handle a shutdown message
        elif data == SHUTDOWN_SIG:
			# Turning off red LED
            self.send_cmd("sys set pindig GPIO11 0")
            
            # Sending staggered response
            RadioResponseFirstDelay()
            self.send_cmd('radio tx '+SHUTDOWN_HEX, delay=0)
            RadioResponseSecondDelay()
            
            # Setting trigger flag
            CMD_SHUTDOWN = 1            
            
        # Handle a ping message
        elif data == PING_SIG:
            
            for a in range(10):
                GPIO.output(PRIMED_LED,GPIO.HIGH)
                time.sleep(.1)
                GPIO.output(PRIMED_LED,GPIO.LOW)
                GPIO.output(RECORDING_LED,GPIO.HIGH)
                time.sleep(.1)
                GPIO.output(RECORDING_LED,GPIO.LOW)
                GPIO.output(COMPLETE_LED,GPIO.HIGH)
                time.sleep(.1)
                GPIO.output(COMPLETE_LED,GPIO.LOW)   
                time.sleep(.25)        
                
            GPIO.output(PRIMED_LED,GPIO.HIGH)
            
            FileCounter = len(glob.glob1(mypath,"*.csv"))
            
            # Response transmitted by radio when the ping message has been
			# received. **MUST STAGGER RESPONSES FROM MULTIPLE RADIOS
            PING_RESPONSE = DAQ_NAME + ' Png' + str(FileCounter).zfill(2) + '.' + str(int(recording_length)).zfill(2)
            PING_HEX = binascii.hexlify(PING_RESPONSE.encode()).decode()
                
            # Sending staggered response
            RadioResponseFirstDelay()
            self.send_cmd('radio tx '+PING_HEX, delay=0)
            RadioResponseSecondDelay()        
            
            self.send_cmd('radio rx 0') # Re-engages continuous reception mode
            
        # Handle a change recording length message
        elif data[0:20] == RECORDINGLENGTH_SIG:
            
            print('     RECV: '+binascii.unhexlify(data[10:]).decode())   
                           
            print('     REC Length: ' + binascii.unhexlify(data[10:]).decode()[6:] )
            
            hat.a_in_scan_cleanup()     
            
            recording_length = int(binascii.unhexlify(data[10:]).decode()[6:])
            
            global samples_per_channel
            samples_per_channel = int(recording_length*actual_scan_rate)
            
            hat.a_in_scan_start(channel_mask, samples_per_channel, scan_rate, options)
            
            self.send_cmd('radio rx 0') # Re-engages continuous reception mode
        
        # Prepare to receive another message
        else:
            self.send_cmd('radio rx 0') # Re-engages continuous reception mode

    def connection_lost(self, exc):
        if exc:
            print(exc)
        print("     port closed")

    def send_cmd(self, cmd, delay=.5):
        self.transport.write(('%s\r\n' % cmd).encode('UTF-8'))
        time.sleep(delay)

def main():
    """
    This function is executed automatically when the module is run directly.
    """
    # Initialize the pins, and set their numbering scheme
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PRIMED_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RECORDING_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(COMPLETE_LED, GPIO.OUT, initial=GPIO.LOW)
    # Convert the list to a channel mask that
    # can be passed as a parameter to the MCC 118 functions.
    global channel_mask
    channel_mask = chan_list_to_mask(channels)
    num_channels = len(channels)
    global options
    options = OptionFlags.EXTTRIGGER # Commands MCC118 to wait for signal on trigger input pin before recording
    trigger_mode = TriggerModes.ACTIVE_HIGH # Commands MCC118 to look for HIGH signal on trigger input pin

    try:
        # Select an MCC 118 HAT device to use.
        address = select_hat_device(HatIDs.MCC_118)
        global hat
        hat = mcc118(address)
        
        # Ready LED
        GPIO.output(PRIMED_LED,GPIO.HIGH)
        
        # Terminal Header
        print('\n\n///////////////////////////////////////////////////////////////////')
        print('\n' + '     ' + DAQ_NAME + ' - Finite Data Acquisition with LoRa Trigger @ 50kHz     \n')
        print('///////////////////////////////////////////////////////////////////')

        global actual_scan_rate
        actual_scan_rate = hat.a_in_scan_actual_rate(num_channels, scan_rate)
        
        global samples_per_channel        
        samples_per_channel = int(recording_length*actual_scan_rate)
        
        # Scan Information to Terminal
        print('\n\n*************************************')
        print('\nSelected Parameters:')
        print('    Channels: ', end='')
        print(', '.join([str(chan) for chan in channels]))
        print('    Requested scan rate (samples/sec/channel): ', scan_rate)
        print('    Actual scan rate (samples/sec/channel): ', actual_scan_rate)
        print('    Options: ', enum_mask_to_string(OptionFlags, options))
        print('    Trigger Mode: ', trigger_mode.name)
        print('    Number of samples/channel requested: ', samples_per_channel)
        print('    Length of recording (seconds): ', samples_per_channel/actual_scan_rate)
        print('    CSV Storage location: ' + mypath)
        print('    DAQ Name:   ' + DAQ_NAME)
        print('    DAQ Number: ', DAQ_NUM)
        print('    Total Number of DAQS: ', NUM_OF_DAQS)
        print('    Radio Response Delay Window (seconds): ', RESPONSE_DELAY)
        print('    Current date/time: ',datetime.strftime(datetime.now(), "%m_%d_%Y, %H:%M:%S"))
        print('\n*************************************')

        hat.trigger_mode(trigger_mode)

        # Prepare MCC118 to start the scan based on above settings.
        hat.a_in_scan_start(channel_mask, samples_per_channel, scan_rate,
                            options)
        try:
            # Wait for the external trigger to occur
            wait_for_trigger(hat)

            print('\n (1) Scanning ... Press Ctrl-C to stop')
            
            # Read and save data from MCC118 as it records
            #read_and_display_data(hat, samples_per_channel, num_channels,fileName)
            
            read_and_display_data(hat, samples_per_channel, num_channels)

        except KeyboardInterrupt:
            # Clear the '^C' from the display.
            print(CURSOR_BACK_2, ERASE_TO_END_OF_LINE, '\n')
            hat.a_in_scan_stop()
            GPIO.cleanup()
            quit()
            
    except (HatError, ValueError, KeyboardInterrupt) as err:
        print('\n', err)
        GPIO.cleanup()
        hat.a_in_scan_stop()

def wait_for_trigger(hat):
    """
    Monitor the status of the specified HAT device in a loop until the
    triggered status is True or the running status is False.

    Args:
        hat (mcc118): The mcc118 HAT device object on which the status will
            be monitored.

    Returns:
        None

    """
    try: 
        # GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        print('\n <<<READY>>>\n\n (0) Waiting for trigger to initiate recording (or press Ctrl+C to abort)\n')
        
        # Wait until LoStik is properly inserted
        global LoStikInserted
        while(LoStikInserted):
            try:
                ser = serial.Serial("/dev/ttyUSB0", baudrate=57600)
                manager = (ReaderThread(ser, PrintLines))
                enter = type(manager).__enter__
                exit = type(manager).__exit__
                value = enter(manager)
                hit_except = False
                LoStikInserted = 0
				
            except:
                for a in range(10):
                    GPIO.output(RECORDING_LED,GPIO.HIGH)
                    time.sleep(.1)
                    GPIO.output(RECORDING_LED,GPIO.LOW)  
                    time.sleep(.1)			
                print("     LoStik USB not Properly Inserted!")         
        LoStikInserted = 1
        try:
            protocol = value
            while(CMD_RECEIVED):
                if GPIO.input(PWR_PIN) == 1 or CMD_SHUTDOWN:
                    print("     Shutting Down")
                    GPIO.cleanup()
                    time.sleep(1)
                    hat.a_in_scan_cleanup()
                    time.sleep(1)
                    quit()
                pass
        except:
            hit_except = True
            if not exit(manager, *sys.exc_info()):
                raise
        finally:
            if not hit_except:
                exit(manager, None, None, None)

    except KeyboardInterrupt:
	    print(CURSOR_BACK_2, ERASE_TO_END_OF_LINE, '\n')
	    hat.a_in_scan_cleanup()
	    GPIO.cleanup()
	    quit()

    # Sends trigger pin on RPi to HIGH which should be connected to MCC118 trigger input pin
    GPIO.setup(TRIGGER_PIN, GPIO.OUT)	
    GPIO.output(TRIGGER_PIN,GPIO.HIGH)
    
    # Read the status only to determine when the trigger occurs.
    is_running = True
    is_triggered = False
    while is_running and not is_triggered:
        status = hat.a_in_scan_status()
        is_running = status.running
        is_triggered = status.triggered
        if not is_triggered:
            time.sleep(0.001)
    GPIO.cleanup()

def read_and_display_data(hat, samples_per_channel, num_channels):
    """
    Reads data from the specified channels on the specified DAQ HAT devices,
    and writes the data to a .csv file.  The reads are executed in a 
    loop that continues until the user stops the scan, the specified 
    scan length is reached, or an overrun error is detected.

    Args:
        hat (mcc118): The mcc118 HAT device object.
        num_channels (int): The number of channels to display.

    Returns:
        None

    """   
    
    # Read all of the available samples (up to the size of the read_buffer which
    # is specified by the user_buffer_size).  Since the read_request_size is set
    # to -1 (READ_ALL_AVAILABLE), this function returns immediately with
    # whatever samples are available (up to user_buffer_size) and the timeout
    # parameter is ignored.
    total_samples_read = 0
    read_request_size = READ_ALL_AVAILABLE
    completeFlag = 0    
    timeout = 5.0
    
    # file switch:  w =  Write to a file
    # file switch:  w+ = Write to a file, if it doesn't exist create it
    # file switch:  a =  Append to a file
    # file switch:  a+ = Append to a file, if is doesn't exist create it.
    # file switch:  x = will create a file, returns an error if the file exist
    

    # If the scan starts, create a file name based upon current date and time.
    # Retrieve the Current Working Directory and generate the full path 
    # to where to write the collected data as a .csv file.  Open the file 
    # begin writing the data to the file.  When done, close the file.
    
    try:
        if os.path.exists(basepath):
            if not (os.path.exists(mypath)):
                os.mkdir(mypath)
        else:
            os.mkdir(basepath)
            os.chdir(basepath)
            os.mkdir(mypath)
    except OSError as exc:
        raise
    
    os.chdir(mypath)
    fileDateTime = datetime.strftime(datetime.now(), "(%m_%d_%Y)-(%H-%M-%S)")
    #filePath = mypath + "/" + DAQ_NAME + "_" + fileName + ".csv"
    filePath = mypath + "/" + DAQ_NAME + "_" + fileDateTime + ".csv"
    csvfile = open(filePath, "w+")
    csvwriter = csv.writer(csvfile) 
    
    # Recording LED
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RECORDING_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(RECORDING_LED,GPIO.HIGH)
    
    while total_samples_read < samples_per_channel:
        read_result = hat.a_in_scan_read(read_request_size, timeout)

        # Check for an overrun error
        if read_result.hardware_overrun:
            print('\n\nHardware overrun\n')
            break
        elif read_result.buffer_overrun:
            print('\n\nBuffer overrun\n')
            break
        elif not (read_result.running and completeFlag == 0):
            completeFlag = 1
            print('\n (2) Recording Completed - Buffer Draining')

        samples_read_per_channel = int(len(read_result.data) / num_channels)
        total_samples_read += samples_read_per_channel
        
        totalSamples = len(read_result.data) 

        if samples_read_per_channel > 0:
            index = samples_read_per_channel * num_channels - num_channels
            
            new_index = 0
            myArray=[] #create an empty array
            for i in range(0, totalSamples, num_channels):
                myArray.append([])  #add a row to the array (COLUMN)
                for j in range(num_channels):
					#append a num_channels of data to the array (ROW)
                    myArray[new_index].append(read_result.data[i + j])  
                new_index+=1

            csvwriter.writerows(myArray) #Write the array to file
            csvfile.flush

    # Cleanup
    csvfile.close()  
    print('\n (3) Buffer Drained - Data Saved to CSV File\n')
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    
    # Complete LED
    GPIO.setup(COMPLETE_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(COMPLETE_LED,GPIO.HIGH)
    time.sleep(5)
    GPIO.cleanup()
    hat.a_in_scan_cleanup()
    global CMD_RECEIVED
    CMD_RECEIVED = 1
    
    # Restarts script to prepare for another recording
    main()

if __name__ == '__main__':
    main()
