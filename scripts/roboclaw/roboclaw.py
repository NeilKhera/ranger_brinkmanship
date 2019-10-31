import serial
import threading
import time

class Cmd():
    M1FORWARD = 0
    M1BACKWARD = 1
    SETMINMB = 2
    SETMAXMB = 3
    M2FORWARD = 4
    M2BACKWARD = 5
    M17BIT = 6
    M27BIT = 7
    MIXEDFORWARD = 8
    MIXEDBACKWARD = 9
    MIXEDRIGHT = 10
    MIXEDLEFT = 11
    MIXEDFB = 12
    MIXEDLR = 13
    GETM1ENC = 16
    GETM2ENC = 17
    GETM1SPEED = 18
    GETM2SPEED = 19
    RESETENC = 20
    GETVERSION = 21
    SETM1ENCCOUNT = 22
    SETM2ENCCOUNT = 23
    GETMBATT = 24
    GETLBATT = 25
    SETMINLB = 26
    SETMAXLB = 27
    SETM1PID = 28
    SETM2PID = 29
    GETM1ISPEED = 30
    GETM2ISPEED = 31
    M1DUTY = 32
    M2DUTY = 33
    MIXEDDUTY = 34
    M1SPEED = 35
    M2SPEED = 36
    MIXEDSPEED = 37
    M1SPEEDACCEL = 38
    M2SPEEDACCEL = 39
    MIXEDSPEEDACCEL = 40
    M1SPEEDDIST = 41
    M2SPEEDDIST = 42
    MIXEDSPEEDDIST = 43
    M1SPEEDACCELDIST = 44
    M2SPEEDACCELDIST = 45
    MIXEDSPEEDACCELDIST = 46
    GETBUFFERS = 47
    GETPWMS = 48
    GETCURRENTS = 49
    MIXEDSPEED2ACCEL = 50
    MIXEDSPEED2ACCELDIST = 51
    M1DUTYACCEL = 52
    M2DUTYACCEL = 53
    MIXEDDUTYACCEL = 54
    READM1PID = 55
    READM2PID = 56
    SETMAINVOLTAGES = 57
    SETLOGICVOLTAGES = 58
    GETMINMAXMAINVOLTAGES = 59
    GETMINMAXLOGICVOLTAGES = 60
    SETM1POSPID = 61
    SETM2POSPID = 62
    READM1POSPID = 63
    READM2POSPID = 64
    M1SPEEDACCELDECCELPOS = 65
    M2SPEEDACCELDECCELPOS = 66
    MIXEDSPEEDACCELDECCELPOS = 67
    SETM1DEFAULTACCEL = 68
    SETM2DEFAULTACCEL = 69
    SETPINFUNCTIONS = 74
    GETPINFUNCTIONS = 75
    SETDEADBAND = 76
    GETDEADBAND = 77
    RESTOREDEFAULTS = 80
    GETTEMP = 82
    GETTEMP2 = 83
    GETERROR = 90
    GETENCODERMODE = 91
    SETM1ENCODERMODE = 92
    SETM2ENCODERMODE = 93
    WRITENVM = 94
    READNVM = 95
    SETCONFIG = 98
    GETCONFIG = 99
    SETM1MAXCURRENT = 133
    SETM2MAXCURRENT = 134
    GETM1MAXCURRENT = 135
    GETM2MAXCURRENT = 136
    SETPWMMODE = 148
    GETPWMMODE = 149
    FLAGBOOTLOADER = 255


class RoboClaw(object):
    def __init__(self, port, rate, address=0x80):
        self._serial = serial.Serial(port, baudrate=rate, timeout=0.1, interCharTimeout=0.01)
        self._address = address
        self._num_tries = 3
        self._lock = threading.Lock()

    def kill(self):
        self._write1(Cmd.M1FORWARD, 0)
        self._write1(Cmd.M2FORWARD, 0)
        self._serial.close()

    def _crc_clear(self):
        self._crc = 0
        return

    def _crc_update(self, data):
        self._crc = self._crc ^ (data << 8)
        for bit in range(0, 8):
            if (self._crc&0x8000)  == 0x8000:
                self._crc = ((self._crc << 1) ^ 0x1021)
            else:
                self._crc = self._crc << 1
        return

    def _sendcommand(self, command):
        self._crc_clear()
        self._crc_update(self._address)
        self._serial.write(chr(self._address))
        self._crc_update(command)
        self._serial.write(chr(command))
        return

    def _readchecksumword(self):
        data = self._serial.read(2)
        if len(data)==2:
            crc = (ord(data[0])<<8) | ord(data[1])
            return (1,crc)
        return (0,0)

    def _readbyte(self):
        data = self._serial.read(1)
        if len(data):
            val = ord(data)
            self._crc_update(val)
            return (1,val)
        return (0,0)

    def _readword(self):
        val1 = self._readbyte()
        if val1[0]:
            val2 = self._readbyte()
            if val2[0]:
                return (1,val1[1]<<8|val2[1])
        return (0,0)

    def _readlong(self):
        val1 = self._readbyte()
        if val1[0]:
            val2 = self._readbyte()
            if val2[0]:
                val3 = self._readbyte()
                if val3[0]:
                    val4 = self._readbyte()
                    if val4[0]:
                        return (1,val1[1]<<24|val2[1]<<16|val3[1]<<8|val4[1])
        return (0,0)

    def _readslong(self):
        val = self._readlong()
        if val[0]:
            if val[1]&0x80000000:
                return (val[0],val[1]-0x100000000)
            return (val[0],val[1])
        return (0,0)

    def _writebyte(self, val):
        self._crc_update(val&0xFF)
        self._serial.write(chr(val&0xFF))

    def _writesbyte(self, val):
        self._writebyte(val)

    def _writeword(self, val):
        self._writebyte((val>>8)&0xFF)
        self._writebyte(val&0xFF)

    def _writesword(self, val):
        self._writeword(val)

    def _writelong(self, val):
        self._writebyte((val>>24)&0xFF)
        self._writebyte((val>>16)&0xFF)
        self._writebyte((val>>8)&0xFF)
        self._writebyte(val&0xFF)

    def _writeslong(self, val):
        self._writelong(val)

    def _read1(self, cmd):
        for _ in xrange(self._num_tries):
            self._serial.flushInput()
            self._sendcommand(cmd)
            val1 = self._readbyte()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc&0xFFFF!=crc[1]&0xFFFF:
                        return (0,0)
                    return (1,val1[1])
        return (0,0)

    def _read2(self, cmd):
        for _ in xrange(self._num_tries):
            self._serial.flushInput()
            self._sendcommand(cmd)
            val1 = self._readword()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc&0xFFFF!=crc[1]&0xFFFF:
                        return (0,0)
                    return (1,val1[1])
        return (0,0)

    def _read4(self, cmd):
        for _ in xrange(self._num_tries):
            self._serial.flushInput()
            self._sendcommand(cmd)
            val1 = self._readlong()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0]:
                    if self._crc&0xFFFF!=crc[1]&0xFFFF:
                        return (0,0)
                    return (1,val1[1])
        return (0,0)

    def _read4_1(self, cmd):
        for _ in xrange(self._num_tries):
            self._serial.flushInput()
            self._sendcommand(cmd)
            val1 = self._readslong()
            if val1[0]:
                val2 = self._readbyte()
                if val2[0]:
                    crc = self._readchecksumword()
                    if crc[0]:
                        if self._crc&0xFFFF!=crc[1]&0xFFFF:
                            return (0,0)
                        return (1,val1[1],val2[1])
        return (0,0)

    def _read_n(self, cmd, args):
        for _ in xrange(self._num_tries):
            self._serial.flushInput()
            failed=False
            self._sendcommand(cmd)
            data = [1,]
            for i in range(0,args):
                val = self._readlong()
                if val[0]==0:
                    failed=True
                    break
                data.append(val[1])
            if failed:
                continue
            crc = self._readchecksumword()
            if crc[0]:
                if self._crc&0xFFFF==crc[1]&0xFFFF:
                    return (data);
        return (0,0,0,0,0)

    def _writechecksum(self):
        self._writeword(self._crc&0xFFFF)
        val = self._readbyte()
        if val[0]:
            return True
        return False

    def _write0(self, cmd):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            if self._writechecksum():
                return True
        return False

    def _write1(self, cmd, val):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writebyte(val)
            if self._writechecksum():
                return True
        return False

    def _write111(self, cmd, val1, val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writebyte(val1)
            self._writebyte(val2)
            if self._writechecksum():
                return True
        return False

    def _write111(self, cmd, val1, val2, val3):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writebyte(val1)
            self._writebyte(val2)
            self._writebyte(val3)
            if self._writechecksum():
                return True
        return False

    def _write2(self, cmd, val):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writeword(val)
            if self._writechecksum():
                return True
        return False

    def _writeS2(self, cmd, val):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writesword(val)
            if self._writechecksum():
                return True
        return False

    def _write22(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writeword(val1)
            self._writeword(val2)
            if self._writechecksum():
                return True
        return False

    def _writeS22(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writesword(val1)
            self._writeword(val2)
            if self._writechecksum():
                return True
        return False

    def _writeS2S2(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writesword(val1)
            self._writesword(val2)
            if self._writechecksum():
                return True
        return False

    def _writeS24(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writesword(val1)
            self._writelong(val2)
            if self._writechecksum():
                return True
        return False

    def _writeS24S24(self,cmd,val1,val2,val3,val4):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writesword(val1)
            self._writelong(val2)
            self._writesword(val3)
            self._writelong(val4)
            if self._writechecksum():
                return True
        return False

    def _write4(self, cmd, val):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val)
            if self._writechecksum():
                return True
        return False

    def _writeS4(self, cmd, val):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writeslong(val)
            if self._writechecksum():
                return True
        return False

    def _write44(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writelong(val2)
            if self._writechecksum():
                return True
        return False

    def _write4S4(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writeslong(val2)
            if self._writechecksum():
                return True
        return False

    def _writeS4S4(self,cmd,val1,val2):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writeslong(val1)
            self._writeslong(val2)
            if self._writechecksum():
                return True
        return False

    def _write441(self,cmd,val1,val2,val3):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writebyte(val3)
            if self._writechecksum():
                return True
        return False

    def _writeS441(self,cmd,val1,val2,val3):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writeslong(val1)
            self._writelong(val2)
            self._writebyte(val3)
            if self._writechecksum():
                return True
        return False

    def _write4S4S4(self,cmd,val1,val2,val3):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writeslong(val3)
            if self._writechecksum():
                return True
        return False

    def _write4S441(self,cmd,val1,val2,val3,val4):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writebyte(val4)
            if self._writechecksum():
                return True
        return False

    def _write4444(self,cmd,val1,val2,val3,val4):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            if self._writechecksum():
                return True
        return False

    def _write4S44S4(self,cmd,val1,val2,val3,val4):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writeslong(val4)
            if self._writechecksum():
                return True
        return False

    def _write44441(self,cmd,val1,val2,val3,val4,val5):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writebyte(val5)
            if self._writechecksum():
                return True
        return False

    def _writeS44S441(self,cmd,val1,val2,val3,val4,val5):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writeslong(val1)
            self._writelong(val2)
            self._writeslong(val3)
            self._writelong(val4)
            self._writebyte(val5)
            if self._writechecksum():
                return True
        return False

    def _write4S44S441(self,cmd,val1,val2,val3,val4,val5,val6):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writeslong(val4)
            self._writelong(val5)
            self._writebyte(val6)
            if self._writechecksum():
                return True
        return False

    def _write4S444S441(self,cmd,val1,val2,val3,val4,val5,val6,val7):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writeslong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writeslong(val5)
            self._writelong(val6)
            self._writebyte(val7)
            if self._writechecksum():
                return True
        return False

    def _write4444444(self,cmd,val1,val2,val3,val4,val5,val6,val7):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writelong(val5)
            self._writelong(val6)
            self._writelong(val7)
            if self._writechecksum():
                return True
        return False

    def _write444444441(self,cmd,val1,val2,val3,val4,val5,val6,val7,val8,val9):
        for _ in xrange(self._num_tries):
            self._sendcommand(cmd)
            self._writelong(val1)
            self._writelong(val2)
            self._writelong(val3)
            self._writelong(val4)
            self._writelong(val5)
            self._writelong(val6)
            self._writelong(val7)
            self._writelong(val8)
            self._writebyte(val9)
            if self._writechecksum():
                return True
        return False

    def ForwardM1(self, val):
        with self._lock:
            return self._write1(Cmd.M1FORWARD,val)

    def BackwardM1(self, val):
        with self._lock:
            return self._write1(Cmd.M1BACKWARD,val)

    def SetMinVoltageMainBattery(self, val):
        with self._lock:
            return self._write1(Cmd.SETMINMB,val)

    def SetMaxVoltageMainBattery(self, val):
        with self._lock:
            return self._write1(Cmd.SETMAXMB,val)

    def ForwardM2(self, val):
        with self._lock:
            return self._write1(Cmd.M2FORWARD,val)

    def BackwardM2(self, val):
        with self._lock:
            return self._write1(Cmd.M2BACKWARD,val)

    def ForwardBackwardM1(self, val):
        with self._lock:
            return self._write1(Cmd.M17BIT,val)

    def ForwardBackwardM2(self, val):
        with self._lock:
            return self._write1(Cmd.M27BIT,val)

    def ForwardMixed(self, val):
        with self._lock:
            return self._write1(Cmd.MIXEDFORWARD,val)

    def BackwardMixed(self, val):
        with self._lock:
            return self._write1(Cmd.MIXEDBACKWARD,val)

    def TurnRightMixed(self, val):
        with self._lock:
            return self._write1(Cmd.MIXEDRIGHT,val)

    def TurnLeftMixed(self, val):
        with self._lock:
            return self._write1(Cmd.MIXEDLEFT,val)

    def ForwardBackwardMixed(self, val):
        with self._lock:
            return self._write1(Cmd.MIXEDFB,val)

    def LeftRightMixed(self, val):
        with self._lock:
            return self._write1(Cmd.MIXEDLR,val)

    def ReadEncM1(self):
        with self._lock:
            return self._read4_1(Cmd.GETM1ENC)

    def ReadEncM2(self):
        with self._lock:
            return self._read4_1(Cmd.GETM2ENC)

    def ReadSpeedM1(self):
        with self._lock:
            return self._read4_1(Cmd.GETM1SPEED)

    def ReadSpeedM2(self):
        with self._lock:
            return self._read4_1(Cmd.GETM2SPEED)

    def ResetEncoders(self):
        with self._lock:
            return self._write0(Cmd.RESETENC)

    def ReadVersion(self):
        with self._lock:
            for _ in xrange(self._num_tries):
                self._serial.flushInput()
                self._sendcommand(Cmd.GETVERSION)
                str = ""
                passed = True
                for i in range(0,48):
                    data = self._serial.read(1)
                    if len(data):
                        val = ord(data)
                        self._crc_update(val)
                        if(val==0):
                            break
                        str+=data[0]
                    else:
                        passed = False
                        break
                if passed:
                    crc = self._readchecksumword()
                    if crc[0]:
                        if self._crc&0xFFFF==crc[1]&0xFFFF:
                            return (1,str)
                        else:
                            time.sleep(0.01)
            return (0,0)

    def SetEncM1(self, cnt):
        with self._lock:
            return self._write4(Cmd.SETM1ENCCOUNT,cnt)

    def SetEncM2(self, cnt):
        with self._lock:
            return self._write4(Cmd.SETM2ENCCOUNT,cnt)

    def ReadMainBatteryVoltage(self):
        with self._lock:
            return self._read2(Cmd.GETMBATT)

    def ReadLogicBatteryVoltage(self):
        with self._lock:
            return self._read2(Cmd.GETLBATT)

    def SetMinVoltageLogicBattery(self, val):
        with self._lock:
            return self._write1(Cmd.SETMINLB,val)

    def SetMaxVoltageLogicBattery(self, val):
        with self._lock:
            return self._write1(Cmd.SETMAXLB,val)

    def SetM1VelocityPID(self,p,i,d,qpps):
        with self._lock:
            return self._write4444(Cmd.SETM1PID,long(d*65536),long(p*65536),long(i*65536),qpps)

    def SetM2VelocityPID(self,p,i,d,qpps):
        with self._lock:
            return self._write4444(Cmd.SETM2PID,long(d*65536),long(p*65536),long(i*65536),qpps)

    def ReadISpeedM1(self):
        with self._lock:
            return self._read4_1(Cmd.GETM1ISPEED)

    def ReadISpeedM2(self):
        with self._lock:
            return self._read4_1(Cmd.GETM2ISPEED)

    def DutyM1(self, val):
        with self._lock:
            return _simplFunctionS2(Cmd.M1DUTY,val)

    def DutyM2(self, val):
        with self._lock:
            return _simplFunctionS2(Cmd.M2DUTY,val)

    def DutyM1M2(m1,m2):
        with self._lock:
            return self._writeS2S2(Cmd.MIXEDDUTY,m1,m2)

    def SpeedM1(self, val):
        with self._lock:
            return self._writeS4(Cmd.M1SPEED,val)

    def SpeedM2(self, val):
        with self._lock:
            return self._writeS4(Cmd.M2SPEED,val)

    def SpeedM1M2(m1,m2):
        with self._lock:
            return _writeS4S4(Cmd.MIXEDSPEED,m1,m2)

    def SpeedAccelM1(self,accel,speed):
        with self._lock:
            return self._write4S4(Cmd.M1SPEEDACCEL,accel,speed)

    def SpeedAccelM2(self,accel,speed):
        with self._lock:
            return self._write4S4(Cmd.M2SPEEDACCEL,accel,speed)

    def SpeedAccelM1M2(self,accel,speed1,speed2):
        with self._lock:
            return self._write4S4S4(Cmd.M1SPEEDACCEL,accel,speed1,speed2)

    def SpeedDistanceM1(self,speed,distance,buffer):
        with self._lock:
            return self._writeS441(Cmd.M1SPEEDDIST,speed,distance,buffer)

    def SpeedDistanceM2(self,speed,distance,buffer):
        with self._lock:
            return self._writeS441(Cmd.M2SPEEDDIST,speed,distance,buffer)

    def SpeedDistanceM1M2(self,speed1,distance1,speed2,distance2,buffer):
        with self._lock:
            return self._writeS44S441(Cmd.MIXEDSPEEDDIST,speed1,distance1,speed2,distance2,buffer)

    def SpeedAccelDistanceM1(self,accel,speed,distance,buffer):
        with self._lock:
            return self._write4S441(Cmd.M1SPEEDACCELDIST,accel,speed,distance,buffer)

    def SpeedAccelDistanceM2(self,accel,speed,distance,buffer):
        with self._lock:
            return self._write4S441(Cmd.M2SPEEDACCELDIST,accel,speed,distance,buffer)

    def SpeedAccelDistanceM1M2(self,accel,speed1,distance1,speed2,distance2,buffer):
        with self._lock:
            return self._write4S44S441(Cmd.MIXEDSPEED2ACCELDIST,accel,speed1,distance1,speed2,distance2,buffer)

    def ReadBuffers(self):
        with self._lock:
            val = self._read2(Cmd.GETBUFFERS)
            if val[0]:
                return (1,val[1]>>8,val[1]&0xFF)
            return (0,0,0)

    def ReadPWMs(self):
        with self._lock:
            val = self._read4(Cmd.GETPWMS)
            if val[0]:
                pwm1 = val[1]>>16
                pwm2 = val[1]&0xFFFF
                if pwm1&0x8000:
                    pwm1-=0x10000
                if pwm2&0x8000:
                    pwm2-=0x10000
                return (1,pwm1,pwm2)
            return (0,0,0)

    def ReadCurrents(self):
        with self._lock:
            val = self._read4(Cmd.GETCURRENTS)
            if val[0]:
                cur1 = val[1]>>16
                cur2 = val[1]&0xFFFF
                if cur1&0x8000:
                    cur1-=0x10000
                if cur2&0x8000:
                    cur2-=0x10000
                return (1,cur1,cur2)
            return (0,0,0)

    def SpeedAccelM1M2_2(self,accel1,speed1,accel2,speed2):
        with self._lock:
            return self._write4S44S4(Cmd.MIXEDSPEED2ACCEL,accel,speed1,accel2,speed2)

    def SpeedAccelDistanceM1M2_2(self,accel1,speed1,distance1,accel2,speed2,distance2,buffer):
        with self._lock:
            return self._write4S444S441(Cmd.MIXEDSPEED2ACCELDIST,accel1,speed1,distance1,accel2,speed2,distance2,buffer)

    def DutyAccelM1(self,accel,duty):
        with self._lock:
            return self._writeS24(Cmd.M1DUTYACCEL,duty,accel)

    def DutyAccelM2(self,accel,duty):
        with self._lock:
            return self._writeS24(Cmd.M2DUTYACCEL,duty,accel)

    def DutyAccelM1M2(self,accel1,duty1,accel2,duty2):
        with self._lock:
            return self._writeS24S24(Cmd.MIXEDDUTYACCEL,duty1,accel1,duty2,accel2)

    def ReadM1VelocityPID(self):
        with self._lock:
            data = self._read_n(Cmd.READM1PID,4)
            if data[0]:
                data[1]/=65536.0
                data[2]/=65536.0
                data[3]/=65536.0
                return data
            return (0,0,0,0,0)

    def ReadM2VelocityPID(self):
        with self._lock:
            data = self._read_n(Cmd.READM2PID,4)
            if data[0]:
                data[1]/=65536.0
                data[2]/=65536.0
                data[3]/=65536.0
                return data
            return (0,0,0,0,0)

    def SetMainVoltages(self, min, max):
        with self._lock:
            return self._write22(Cmd.SETMAINVOLTAGES,min,max)

    def SetLogicVoltages(self, min, max):
        with self._lock:
            return self._write22(Cmd.SETLOGICVOLTAGES,min,max)

    def ReadMinMaxMainVoltages(self):
        with self._lock:
            val = self._read4(Cmd.GETMINMAXMAINVOLTAGES)
            if val[0]:
                min = val[1]>>16
                max = val[1]&0xFFFF
                return (1,min,max)
            return (0,0,0)

    def ReadMinMaxLogicVoltages(self):
        with self._lock:
            val = self._read4(Cmd.GETMINMAXLOGICVOLTAGES)
            if val[0]:
                min = val[1]>>16
                max = val[1]&0xFFFF
                return (1,min,max)
            return (0,0,0)

    def SetM1PositionPID(self,kp,ki,kd,kimax,deadzone,min,max):
        with self._lock:
            return self._write4444444(Cmd.SETM1POSPID,long(kd*1024),long(kp*1024),long(ki*1024),kimax,deadzone,min,max)

    def SetM2PositionPID(self,kp,ki,kd,kimax,deadzone,min,max):
        with self._lock:
            return self._write4444444(Cmd.SETM2POSPID,long(kd*1024),long(kp*1024),long(ki*1024),kimax,deadzone,min,max)

    def ReadM1PositionPID(self):
        with self._lock:
            data = self._read_n(Cmd.READM1POSPID,7)
            if data[0]:
                data[0]/=1024.0
                data[1]/=1024.0
                data[2]/=1024.0
                return data
            return (0,0,0,0,0,0,0,0)

    def ReadM2PositionPID(self):
        with self._lock:
            data = self._read_n(Cmd.READM2POSPID,7)
            if data[0]:
                data[0]/=1024.0
                data[1]/=1024.0
                data[2]/=1024.0
                return data
            return (0,0,0,0,0,0,0,0)

    def SpeedAccelDeccelPositionM1(self,accel,speed,deccel,position,buffer):
        with self._lock:
            return self._write44441(Cmd.M1SPEEDACCELDECCELPOS,accel,speed,deccel,position,buffer)

    def SpeedAccelDeccelPositionM2(self,accel,speed,deccel,position,buffer):
        with self._lock:
            return self._write44441(Cmd.M2SPEEDACCELDECCELPOS,accel,speed,deccel,position,buffer)

    def SpeedAccelDeccelPositionM1M2(self,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer):
        with self._lock:
            return self._write444444441(Cmd.MIXEDSPEEDACCELDECCELPOS,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer)

    def SetM1DefaultAccel(self,accel):
        with self._lock:
            return self._write4(Cmd.SETM1DEFAULTACCEL,accel)

    def SetM2DefaultAccel(self,accel):
        with self._lock:
            return self._write4(Cmd.SETM2DEFAULTACCEL,accel)

    def SetPinFunctions(self,S3mode,S4mode,S5mode):
        with self._lock:
            return self._write111(Cmd.SETPINFUNCTIONS,S3mode,S4mode,S5mode)

    def ReadPinFunctions(self):
        with self._lock:
            for _ in xrange(self._num_tries):
                self._sendcommand(Cmd.GETPINFUNCTIONS)
                val1 = self._readbyte()
                if val1[0]:
                    val2 = self._readbyte()
                    if val1[0]:
                        val3 = self._readbyte()
                        if val1[0]:
                            crc = self._readchecksumword()
                            if crc[0]:
                                if self._crc&0xFFFF!=crc[1]&0xFFFF:
                                    return (0,0)
                                return (1,val1[1],val2[1],val3[1])
            return (0,0)

    def SetDeadBand(min,max):
        with self._lock:
            return self._write111(Cmd.SETDEADBAND,min,max)

    def GetDeadBand(self):
        with self._lock:
            val = self._read2(Cmd.GETDEADBAND)
            if val[0]:
                return (1,val[1]>>8,val[1]&0xFF)
            return (0,0,0)

    #Warning(TTL Serial): Baudrate will change if not already set to 38400.  Communications will be lost
    def RestoreDefaults(self):
        with self._lock:
            return self._write0(Cmd.RESTOREDEFAULTS)

    def ReadTemp(self):
        with self._lock:
            return self._read2(Cmd.GETTEMP)

    def ReadTemp2(self):
        with self._lock:
            return self._read2(Cmd.GETTEMP2)

    def ReadError(self):
        with self._lock:
            return self._read2(Cmd.GETERROR)

    def ReadEncoderModes(self):
        with self._lock:
            val = self._read2(Cmd.GETENCODERMODE)
            if val[0]:
                return (1,val[1]>>8,val[1]&0xFF)
            return (0,0,0)

    def SetM1EncoderMode(self, mode):
        with self._lock:
            return self._write1(Cmd.SETM1ENCODERMODE,mode)

    def SetM2EncoderMode(self, mode):
        with self._lock:
            return self._write1(Cmd.SETM2ENCODERMODE,mode)

    #saves active settings to NVM
    def WriteNVM(self):
        with self._lock:
            return self._write4(Cmd.WRITENVM,0xE22EAB7A)

    #restores settings from NVM
    #Warning(TTL Serial): If baudrate changes or the control mode changes communications will be lost
    def ReadNVM(self):
        with self._lock:
            return self._write0(Cmd.READNVM)

    #Warning(TTL Serial): If control mode is changed from packet serial mode when setting config communications will be lost!
    #Warning(TTL Serial): If baudrate of packet serial mode is changed communications will be lost!
    def SetConfig(config):
        with self._lock:
            return self._write2(Cmd.SETCONFIG,config)

    def GetConfig(self):
        with self._lock:
            return self._read2(Cmd.GETCONFIG)

    def SetM1MaxCurrent(self, max):
        with self._lock:
            return self._write44(Cmd.SETM1MAXCURRENT,max,0)

    def SetM2MaxCurrent(self, max):
        with self._lock:
            return self._write44(Cmd.SETM2MAXCURRENT,max,0)

    def ReadM1MaxCurrent(self):
        with self._lock:
            data = self._read_n(Cmd.GETM1MAXCURRENT,2)
            if data[0]:
                return (1,data[1])
            return (0,0)

    def ReadM2MaxCurrent(self):
        with self._lock:
            data = self._read_n(Cmd.GETM2MAXCURRENT,2)
            if data[0]:
                return (1,data[1])
            return (0,0)

    def SetPWMMode(mode):
        with self._lock:
            return self._write1(Cmd.SETPWMMODE,mode)

    def ReadPWMMode(self):
        with self._lock:
            return self._read1(Cmd.GETPWMMODE)
