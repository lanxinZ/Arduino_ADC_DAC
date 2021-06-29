# corresponding Arduino code:
# mySketch_jun21a
# R:\myProjects\Arduino\Task_from_Gongmangmang\Arduino_Code

# Change log: add methode 5 for reseting PSU_DAC


import serial
import time

serialPort = 'COM4'
baudRate = 9600
ser = serial.Serial(serialPort, baudRate, timeout=0.5)
print("Serial setup: Port = %s, baudrate = %d" % (serialPort, baudRate))


def resetAll():     #reset all devices besides PSU DAC
    ser.write(b'0000000')


def resetPSUDAC():     #reset all devices besides PSU DAC
    ser.write(b'5000000')


# DAC_channel: 1~8 for DAC1, 9~16 for DAC2; value: 0~5000mV
def setV_channel(DAC_channel, value):
    b_command = b'1'
    b_command += bytes(DAC_channel)
    b_command += bytes(value)
    # print(b_command)
    ser.write(bytes(b_command))


# PSUDAC_channel: 1,2; value: PVout, 0~4800mV
def setPV_channel(DAC_channel, pvout_value):
    b_command = b'2'
    b_command += bytes(DAC_channel)
    b_command += bytes(pvout_value)
    # print(b_command)
    ser.write(bytes(b_command))


# ADC_channel: AIN2~PVsen1, AIN1~PVsen2
def getPV_channel(ADC_channel):
    b_command = b'3'
    b_command += bytes(ADC_channel)
    b_command += bytes(b'0000')
    # print(b_command)
    ser.write(bytes(b_command))


# AIN3~PIsen1, AIN0~PIsen2
def getI_channel(ADC_channel):
    b_command = b'4'
    b_command += bytes(ADC_channel)
    b_command += bytes(b'0000')
    # print(b_command)
    ser.write(bytes(b_command))


# methode 0
def while_resetAll():
    str_read = b'halloworld'
    while str_read != b'Case 0 finished. \n':
        resetAll()
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))
##        print(str_read)
##        if str_read == b'Case 0 finished. \n':
##             break


# methode 5
def while_resetPSUDAC():
    str_read = b'halloworld'
    while str_read != b'Case 5 finished. \n':
        resetPSUDAC()
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))


# methode 1
def while_setV_channel():
    str_read = b'halloworld'
    while str_read != b'Case 1 finished. \n':
        setV_channel(b'01', b'1080')    #each pin measure 1000, 2000mv; acceptable deviation: +-10mV
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))
        # print(str_read)
        # if str_read == b'Case 1 finished. \n':
        #     break


# methode 2
def while_setPV_channel():
    str_read = b'halloworld'
    while str_read != b'Case 2 finished. \n':
        setPV_channel(b'01', b'2000')   #700mV, 2000mV, 3000mV
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))


# methode 3
def while_getPV_channel():
    str_read = b'halloworld'
    while str_read != b'Case 3 finished. \n':
        getPV_channel(b'01')
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))


# methode 4
def while_getI_channel():
    str_read = b'halloworld'
    while str_read != b'Case 4 finished. \n':
        getI_channel(b'00')
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))


if __name__ == "__main__":
##    while_resetAll()
##    print("----------------")
##    while_resetPSUDAC()
##    print("----------------")
##    while_setV_channel()
##    print("----------------")
##    while_setPV_channel()
##    print("----------------")
##    while_getPV_channel()
##    print("----------------")
##    while_getI_channel()
##    print("----------------")
    
    while 1:
        setV_channel(b'22', b'2500')
        str_read = ser.readline()
        print(str_read.decode('UTF-8'))

##    while 1:
##        getPV_channel(b'01')
##        str_read = ser.readline()
##        print(str_read.decode('UTF-8'))
