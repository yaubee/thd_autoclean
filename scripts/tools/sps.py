import snap7
from snap7 import util
import shutil
import time
import subprocess

debug_mode=True
timeout=10.0
ip1="192.168.100.50"



class SPSBase:
    def __init__(self,ip):
        self.ip=ip
        self.client = None
        self.client_write = None
        self.initialize()

    def initialize(self):
        if self.sps_ping():
            self.client = snap7.client.Client()
            self.client_write = snap7.client.Client()
            self.client.connect(self.ip, 0, 1)
            self.client_write.connect(self.ip, 0, 1)
        else:
            print("SPS is not powered or not connected.")
        
    def sps_ping(self):
        response = subprocess.run(['ping', '-c', '1', '-W', '1', self.ip])
        if response.returncode == 0:
            return True
        else:
            return False

    def run_if_sps_okay(func):
        def wrapper(self, *args, **kwargs):
            if self.sps_ping():
                return func(self,*args,**kwargs)
            else:
                print(f"SPS at {self.ip} not connected.")
                return False
        return wrapper

class SPSStrahlkabine(SPSBase):
    def __init__(self,ip):
        super().__init__(ip)
        self.strahlen(1,1,1)


    def strahlen(self, strahl_time=5, strahl_eintritt_time=10, strahl_austritt_time = 5):
        print("Strahlen started...")
        db_read = self.client.db_read(20, 0, 1)
        strahl_begin = util.get_bool(db_read, 0, 0)
        strahl_end = util.get_bool(db_read, 0, 1)
        print(f"Bool: begin: {strahl_begin}; end: {strahl_end}.")

        state = True # Strahlkabine Enable. Should always be ON, else OFF will shutdown everything
        data = bytearray(1)
        util.set_bool(data,0,0,state)
        self.client_write.db_write(21, 0, data)

        state = False # Strahlkabine Start Sequence. (Switch to 0 when strahl_begin feedback (0, and then 1 again))
        data = bytearray(1)
        util.set_bool(data,0,0,state)
        self.client_write.db_write(21, 1, data)

        time.sleep(0.5)
        state = True # Strahlkabine Start Sequence. (Switch to 0 when strahl_begin feedback (0, and then 1 again))
        data = bytearray(1)
        util.set_bool(data,0,0,state)
        self.client_write.db_write(21, 1, data)

        data = bytearray(2)
        util.set_int(data, 0, strahl_time) # time in s
        self.client_write.db_write(21, 2, data)

        data = bytearray(2)
        util.set_int(data, 0, strahl_eintritt_time) # time
        self.client_write.db_write(21, 4, data)

        data = bytearray(2)
        util.set_int(data, 0, strahl_austritt_time) # time
        self.client_write.db_write(21, 6, data)
    
    def get_strahlen_begin_status(self):
        db_read = self.client.db_read(20, 0, 1)
        return util.get_bool(db_read, 0, 0) #strahl_begin
    
    def get_strahlen_end_status(self):
        db_read = self.client.db_read(20, 0, 1)
        return util.get_bool(db_read, 0, 1) #strahl_end

class SPSSensor(SPSBase):
    def __init__(self,ip,sensor_number):
        super().__init__(ip)
        if sensor_number>=2:
            self.sensor_num=sensor_number
        else:
            print("Can't assign lower DB value than 2.")

    #@SPSBase.run_if_sps_okay
    def get_sensor_bool(self):
        db_read = self.client.db_read(16, 0, 1) ### Read DB, size 1 Byte
        SensorRead = util.get_bool(db_read, 0, self.sensor_num)
        return not SensorRead

class SPSConveyor(SPSBase):
    def __init__(self,ip, conveyor_number, sensors):
        super().__init__(ip)
        self.antrieb_num=conveyor_number
        self.move_rel_done_num=conveyor_number+1
        self.sensor_in, self.sensor_out=sensors
        # Conveyor with stepper motor: Takes 0,1 in DB 16
        self.antrieb_ready=None
        self.move_rel_done=None
    
    def initialize_conveyor(self):
        self.power_conveyor(False)
        time.sleep(0.5)
        self.power_conveyor(True)
        stime=time.time()
        while True:
            db_read = self.client.db_read(16, 0, 1) ### Read DB 16, size 1 Byte
            self.antrieb_ready = util.get_bool(db_read, 0, self.antrieb_num)
            time.sleep(0.5)
            if self.antrieb_ready:
                if debug_mode:
                    print(f"Antrieb ready: {self.antrieb_ready}")
                return True
            else:
                if (time.time()-stime)>timeout:
                    print(f"Timed out after {timeout} seconds.")
                    self.power_conveyor(False)
                    return False
                else:
                    pass

    def power_conveyor(self, state):
        data = bytearray(1)
        util.set_bool(data,0,0,state) #
        self.client_write.db_write(17, 0, data)
        if debug_mode:
            print(f"Power internal motor drive: {state} ")

    def move_conveyor(self, state, dir=None, vel=100, dist=None):
        data = bytearray(4)
        if dist is None:
            pass
        else:
            util.set_dint(data, 0, dist) # distance
            self.client_write.db_write(17,6,data)
        util.set_dint(data, 0, vel) # velocity
        self.client_write.db_write(17, 10, data)
        time.sleep(0.25)
        if state:
            self.power_conveyor(True)
            time.sleep(0.25)
            if dir=="FWD": 
                print(f"Moving conveyor1 forwards at velocity {vel} mm/s.")
                data = bytearray(1)
                util.set_bool(data,0,0,False) # set conveyor to move
                self.client_write.db_write(17,3, data) # 3 is backward byte
                util.set_bool(data,0,0,True) # set conveyor to move
                self.client_write.db_write(17,2, data) # 2 is forward byte
                
            elif dir=="BWD":
                print(f"Moving conveyor1 backwards at velocity {vel} mm/s.")
                data = bytearray(1)
                util.set_bool(data,0,0,False) # set conveyor to move
                self.client_write.db_write(17,2, data) # 2 is forward byte
                util.set_bool(data,0,0,True) # set conveyor to move
                self.client_write.db_write(17,3, data) # 3 is backward byte
            else:
                print("No direction. Not moving.")
        else:
            print("Stopping conveyor1...")
            data = bytearray(1)
            util.set_bool(data,0,0,False) # set conveyor to move
            self.client_write.db_write(17,2, data) # 2 is forward byte
            util.set_bool(data,0,0,False) # set conveyor to move
            self.client_write.db_write(17,3, data) # 3 is backward byte 
            time.sleep(0.25)
            self.power_conveyor(False)

        #data = bytearray(1)
        #util.set_bool(data,0,0,state) # set conveyor to move
        #self.client.db_write(17,2, data)

    def move_conveyor_1000(self):
        data = bytearray(4)
        util.set_dint(data, 0, 1000) # distance
        self.client_write.db_write(17,6,data)
        util.set_dint(data, 0, 100) # velocity
        self.client_write.db_write(17, 10, data)

        time.sleep(0.25)
        self.power_conveyor(True)
        data = bytearray(1)
        util.set_bool(data,0,0,False) # set conveyor to move
        self.client_write.db_write(17,2, data) # 2 is forward byte
        util.set_bool(data,0,0,True) # set conveyor to move
        self.client_write.db_write(17,3, data) # 3 is backward byte

    def simple_move(self):
        if self.initialize_conveyor():
            stime=time.time()
            self.move_conveyor(state=True,dir="FWD", dist=100)
            while (time.time()-stime)<timeout/3:
                if (time.time()-stime)>=2:
                    time.sleep(0.1)
                    self.move_conveyor(False)
                else:
                    continue

    # move conveyor just enough to drop N parts.N=1 by default.
    def move_conveyor_till_drop(self, sensor_to_track=None, number_of_parts=1):
        if sensor_to_track is None:
            sensor_to_track=self.sensor_out
        self.move_conveyor(state=True, dir="FWD")
        st=time.time()
        sensor_previous_state=sensor_to_track.get_sensor_bool()
        parts_count=0
        if debug_mode:
            print(f"Current sensor_out state: {sensor_previous_state}")
        if sensor_previous_state is False:
            while (time.time()-st)<10:
                if debug_mode:
                    print(f"Current sensor_out state: {sensor_to_track.get_sensor_bool()} - Previous state: {sensor_previous_state}")
                if sensor_to_track.get_sensor_bool() and not sensor_previous_state:
                    parts_count+=1
                    if debug_mode:
                        print(f"Parts: {parts_count}")
                sensor_previous_state=sensor_to_track.get_sensor_bool()
                
                if parts_count>=number_of_parts:
                    if debug_mode:
                        print(f"Parts: {parts_count}")
                    time.sleep(2)
                    self.move_conveyor(state=False)
                    break
                if debug_mode:
                    print(f"Parts: {parts_count}")
        elif sensor_previous_state is True:
            self.simple_move()
        if debug_mode:
            print(f"Parts: {parts_count} - Loop terminated for parts drop.")
    
    def simple_dump(self):
        # Used to clear the conveyor and count how many items were on
        self.parts_dumped=0
        if self.initialize_conveyor():
            stime=time.time()
            if self.sensor_out.get_sensor_bool():
                # perform dump
                self.move_conveyor_1000()
                time.sleep(0.1)
                c2_previous=self.sensor_out.get_sensor_bool()
                self.parts_dumped+=1
                # stay in while loop
                while True:
                    if self.sensor_out.get_sensor_bool() and not c2_previous:
                        self.parts_dumped+=1
                        if debug_mode:
                            print(f"Parts added to Strahlkabine:  {self.parts_dumped}")
                    c2_previous=self.sensor_out.get_sensor_bool()
                    if (time.time()-stime)>10:
                        time.sleep(0.1)
                        self.move_conveyor(False)
                        print(f"Total parts added: {self.parts_dumped}")
                # count parts
                # stop moving after set time (break out of loop)
                # save number of parts dumped
            else:
                # Nothing is present on conveyor
                print(f"Nothing to dump. Conveyor empty.")
                print(f"Sensor_IN: {self.sensor_in.get_sensor_bool()} \n Sensor_OUT: {self.sensor_out.get_sensor_bool()}")

class SPSVibratingPlate(SPSBase):
    def __init__(self,ip):
        super().__init__(ip)
        if self.sps_ping():
            db_read = self.client.db_read(18, 0, 6) # Read DB 18, 0 to read the whole DB, size 6 Byte 
            self.motor_ok = util.get_bool(db_read, 0, 0) # get_bool(datablock, byte, bit)
            self.motor_freq = util.get_dint(db_read, 2) # get_double integer (datablock, byte)
        else:
            self.motor_ok=False
        self.motor_status=False
        self.motor_frequency=0
        self.motor_direction=None
        # direction: False --> Forward
        # direction: True --> Reverse
        if self.client is not None:
            self.motor_initialize()

    @SPSBase.run_if_sps_okay
    def motor_initialize(self):
        self.motor_ena_rst_execute(False)
        self.motor_fwd_execute(False)
        self.motor_rev_execute(False)
        stime=time.time()
        print(f"Waiting for motor for {timeout} s.")
        while True:
            if self.motor_check():
                self.motor_ena_rst_execute(True)
                return True
            else:
                self.motor_ena_rst_execute(False)
                time.sleep(0.5)
                if (time.time()-stime)>timeout:
                    print(f"Timed out after {timeout} seconds. \n")
                    print("motor_check Time-out: Press the reset button on the motor driver.")
                    self.motor_ena_rst_execute(False)
                    return False
                else:
                    pass
            time.sleep(0.5)

    @SPSBase.run_if_sps_okay
    def change_motor_freq(self,freq):
        if isinstance(freq, (float, int)):
            freq = str(freq)
            if self.motor_read_freq() == freq:
                print(f"Frequency already at {freq}.")
            else:
                self.motor_read_freq()
        else:
            print("Invalid frequency value. Please provide a float or an integer.")

    @SPSBase.run_if_sps_okay
    def plate_vibrate(self,state,direction):
        self.motor_direction = direction
        # direction: False --> Forward
        # direction: True --> Reverse
        if self.motor_initialize():
            if direction:
                self.motor_fwd_execute(False)
                self.motor_rev_execute(state)
            elif not direction:
                self.motor_fwd_execute(state)
                self.motor_rev_execute(False)
            track=True
        else:
            track=False # This prevents the motor from running continuously in case motor not OK anymore.
        if track and state:
            if direction:
                self.motor_fwd_execute(False)
                self.motor_rev_execute(state)
            elif not direction:
                self.motor_fwd_execute(state)
                self.motor_rev_execute(False)
        else:
            self.change_motor_freq(0)
            if direction:
                self.motor_fwd_execute(False)
                self.motor_rev_execute(state)
            elif not direction:
                self.motor_fwd_execute(state)
                self.motor_rev_execute(False)


    def vibrate_plate(self,state,direction): # direction=False -> Forward, direction=True->reverse
        if self.motor_initialize():
            if direction:
                self.motor_fwd_execute(False)
                self.motor_rev_execute(state)
                #track = True
            else:
                self.motor_fwd_execute(state)
                self.motor_rev_execute(False)
                #track = True
        track = True    
        if track and state:
            if direction:
                self.motor_fwd_execute(False)
                self.motor_rev_execute(state)
            else:
                self.motor_fwd_execute(state)
                self.motor_rev_execute(False)
        else:
            self.change_freq(0)
            if direction:
                self.motor_fwd_execute(False)
                self.motor_rev_execute(state)
            else:
                self.motor_fwd_execute(state)
                self.motor_rev_execute(False)
            time.sleep(0.25)
            pass

    def change_freq(self,freq):
        freq=str(freq)
        if self.motor_read_freq() == freq:
            pass
        else:
            self.motor_speed_ref(freq)

    @SPSBase.run_if_sps_okay
    def motor_fwd_execute(self, state):
        # write MotorFwd byte 0 bit 0 in DB19
        data = bytearray(1)
        util.set_bool(data,0,0,state) # set_bool(datablock, byte, bit, STATE)
        self.client_write.db_write(19, 0, data) # write (datablock, byte, DATA)
        print(f"Motor Forward Execution state: {state}.")

    @SPSBase.run_if_sps_okay
    def motor_ena_rst_execute(self, state):
        # write MotorEnaRst byte 1 bit 0 in DB19
        data = bytearray(1)
        util.set_bool(data,0,0,state)  
        self.client_write.db_write(19, 1, data) 
        if debug_mode:
            print(f"Motor EnaRst (Enable Reset) state: {state}.")

    @SPSBase.run_if_sps_okay
    def motor_rev_execute(self, state):
        # write MotorRev byte 2 bit 0 in DB19
        data = bytearray(1)
        util.set_bool(data,0,0,state) 
        self.client_write.db_write(19, 2, data) 
        print(f"Motor Reverse Execution state: {state}.")

    @SPSBase.run_if_sps_okay
    def motor_speed_ref(self, freq):
        # write MotorSpeedRef byte 4 bit 0 in DB19
        data = bytearray(4)
        util.set_dint(data, 0, freq) # frequency
        self.client_write.db_write(19, 4, data) # write (datablock, byte, DATA)
        print(f"Motor SpeedRef (Speed Reference) state: {freq}.")

    @SPSBase.run_if_sps_okay
    def motor_read_freq(self):
        db_read = self.client.db_read(18, 0, 6) ### Read DB 18, size 6 Byte
        self.motor_frequency = util.get_dint(db_read, 2) # get_double integer (datablock, byte)
        return self.motor_frequency

    @SPSBase.run_if_sps_okay
    def motor_check(self):
        db_read = self.client.db_read(18, 0, 6) ### Read DB 18, size 6 Byte
        self.motor_ok = util.get_bool(db_read, 0, 0)
        return self.motor_ok

if __name__ == "__main__":
    # conveyor: move_conveyor
    # sensor: read_sensor_value
    # vibrating_plate: vibrate_plate

    sensor_number1=2
    sensor_number2=4
    sensor_number3=3
    conveyor_number1=0

    sensor1=SPSSensor(ip1,sensor_number1)
    sensor2=SPSSensor(ip1,sensor_number2)
    sensor3=SPSSensor(ip1,sensor_number3)

    conveyor1=SPSConveyor(ip1,conveyor_number1, [sensor1,sensor2])

    #conveyor1.simple_move()
    while False:
        if sensor1.get_sensor_bool():
            conveyor1.simple_move()
    
    conveyor1.move_conveyor(False)

    """
    vibratingplate=SPSVibratingPlate(ip1)
    vibratingplate.vibrate_plate(False,True)
    vibratingplate.change_freq(10)
    vibratingplate.vibrate_plate(True,True)
    time.sleep(10)
    vibratingplate.vibrate_plate(True,False)
    time.sleep(10)
    vibratingplate.vibrate_plate(False,True)

    
    
    strahlkabine = SPSStrahlkabine(ip1)
    st=time.time()
    while (time.time() - st)<5:
        print(f"Strahlen_begin: {strahlkabine.get_strahlen_begin_status()} - Strahlen_end: {strahlkabine.get_strahlen_end_status()} ")

    strahlkabine.strahlen()
    while True:
        print(f"Strahlen_begin: {strahlkabine.get_strahlen_begin_status()} - Strahlen_end: {strahlkabine.get_strahlen_end_status()} ")
        time.sleep(0.5)

    
    sensor_number1=2
    sensor_number2=4
    sensor_number3=3
    conveyor_number1=0
    sensor1=SPSSensor(ip1,sensor_number1)
    sensor2=SPSSensor(ip1,sensor_number2)
    sensor3=SPSSensor(ip1,sensor_number3)

    while True:
        print(f"sensor1: {sensor1.get_sensor_bool()} - sensor2: {sensor2.get_sensor_bool()} - sensor3: {sensor3.get_sensor_bool()}")
        time.sleep(1)

    
    
    vplate = SPSVibratingPlate(ip1)
    sensor_number1=2
    sensor_number2=3
    conveyor_number1=0
    sensor1=SPSSensor(ip1,sensor_number1)
    sensor2=SPSSensor(ip1,sensor_number2)
    conveyor1=SPSConveyor(ip1,conveyor_number1, [sensor1,sensor2])
    """
    strahlkabine = SPSStrahlkabine(ip1)
    strahlkabine.strahlen()
    
