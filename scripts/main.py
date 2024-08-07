from edge_depth import Detection, GraspableEdge
from check_calibration import DepthCameraApp, InspectionCameraApp
from transformation import RobotControl, ObjectGrasp
from tools.sps import SPSStrahlkabine, SPSSensor, SPSConveyor, SPSVibratingPlate
from tools.ur_realtime import URRealTime
from tools.processes import run_yolo_on_image, save_instance_to_json
import threading, time, queue
from inspection.object_recognition_V1 import object_recognition
from inspection.powder_recognition_V3 import powder_recognition

sps_ip="192.168.100.50"
debug_mode=True
override=False
manual_mode=True

pause_event=threading.Event()


def initialize_autoclean():
    vibratingplate, conveyor1_sensor1, conveyor1_sensor2, conveyor1, strahlkabine, ur5e, depth_cam, inspection_cam = None, None, None, None, None, None, None, None
    # Initialize all devices in order of operations
    print("Starting initialization phase...")
    # 1. Vibrating Plate, conveyor sensors, conveyors, strahlkabine:
    vibratingplate=SPSVibratingPlate(sps_ip)
    sps_status=vibratingplate.sps_ping()
    conveyor1_sensor1=SPSSensor(sps_ip,sensor_number=2)
    conveyor1_sensor2=SPSSensor(sps_ip,sensor_number=4)
    conveyor1=SPSConveyor(sps_ip,conveyor_number=0, sensors=[conveyor1_sensor1,conveyor1_sensor2] )
    strahlkabine=SPSStrahlkabine(sps_ip)

    # 2. Robot at Vibrating Plate Position for Nominal Depth (Depth Camera)
    ur5e=URRealTime()
    ur5e_ready=ur5e.ur_ready()
    if ur5e_ready:
        start_pose=ur5e.ur_checklog()
        end_pose = 3
        ur5e.ur_move_from_A_to_B(start_pose,end_pose)
        confirmrobotposition="n"
        while confirmrobotposition != "y":
            confirmrobotposition=input("Is robot at required vibrating plate position to initialize camera? (y/n)")
            time.sleep(0.1)
        confirmrobotposition="n"
        depth_cam = DepthCameraApp()
        if depth_cam.initialize_camera(): # nominal depth to be used later for conveyor drop
            save_instance_to_json(depth_cam)
            time.sleep(0.1)
            depth_cam_status=True
        else:
            depth_cam_status=False
    
    # 3. Inspection Cameras: 
    inspection_cam=InspectionCameraApp()
    inspection_cam_status=inspection_cam.initialize_camera()

    # Verify response from all devices:
    print("Verifying responses...")
    all_devices_okay=sps_status and ur5e_ready and depth_cam_status and inspection_cam_status

    if all_devices_okay:
        print("System ready to start.")
        return vibratingplate, conveyor1_sensor1, conveyor1_sensor2, conveyor1, strahlkabine, ur5e, depth_cam, inspection_cam
    else:
        print("System NOT ready.")
        print(f"System Report: \nSPS: {sps_status} \nUR5e: {ur5e_ready} \nDepth Camera: {depth_cam_status} \nInspection Camera: {inspection_cam_status}")
        override=input("Do you wish to override the system? y/n \n")
        if override=='y':
            return vibratingplate, conveyor1_sensor1, conveyor1_sensor2, conveyor1, strahlkabine, ur5e, depth_cam, inspection_cam
        else:
            return False

class BlockZeroActions:
    def __init__(self, vibratingplate, freq, vibrating_time, change_dir, vibrating_cycle, variance=10):
        self.vibratingplate=vibratingplate
        self.start_automation=False
        self.emergency_stop=False
        # vibratingplate parameters
        self.vibrating_time=vibrating_time
        self.change_dir=change_dir
        self.vibrating_cycle=vibrating_cycle
        self.freq_threshold=10
        if freq<0 or freq>self.freq_threshold:
            print(f"Please choose a frequency within 0 and {self.freq_threshold}. Frequency defaulted to 10.")
        else:
            self.freq=freq
        if variance > 25:
            self.variance=0
            print("Please choose a frequency variance less than 25%. Frequency variance defaulted to 0.")
        else:
            self.variance=variance
        self.start_automation=True

    def vibrate_sequence(self):
        if self.start_automation:
            dir=True
            self.vibratingplate.vibrate_plate(False,dir)
            while self.start_automation:
                self.vibratingplate.change_freq(freq)
                self.vibratingplate.vibrate_plate(True,dir)
                st=time.time()
                while (time.time()-st)<self.vibrating_time/2:
                    continue
                if self.change_dir:
                    
                    dir=False

                while (time.time()-st)<self.vibrating_time:
                    continue
                self.vibratingplate.change_freq(0)
                self.vibratingplate.vibrate_plate(False,dir)
                self.start_automation=False


class BlockOneActions:
    def __init__(self, conveyor1, strahlkabine, ur5e, sensors):
        self.conveyor1=conveyor1
        self.strahlkabine=strahlkabine
        self.ur5e=ur5e
        self.sensors=sensors
        # Action mappings:
        self.action_map = {
        (False, False, False): self.perform_action_series1,
        (False, False, True): self.perform_action_series2,
        (False, True, False): self.perform_action_series3,
        (False, True, True): self.perform_action_series4,
        (True, False, False): self.perform_action_series5,
        (True, False, True): self.perform_action_series6,
        (True, True, False): self.perform_action_series7,
        (True, True, True): self.perform_action_series8
        }
        self.lock=threading.Lock()
        self.current_action_thread=None
    
    def perform_action_series1(self):
        print("PICK + PLACE 1, no move_conveyor, no strahlen")

    def perform_action_series2(self):
        print("PICK + PLACE 1, no move_conveyor, no strahlen")
        
    def perform_action_series3(self):
        print("no PICK + PLACE 1, simple_move, no strahlen")
        conveyor1.simple_move()

    def perform_action_series4(self):
        print("no PICK + PLACE 1, no move_conveyor, no strahlen")

    def perform_action_series5(self):
        print("PICK + PLACE 1, no move_conveyor, no strahlen")

    def perform_action_series6(self):
        print("PICK + PLACE 1, move_conveyor_till_drop > strahlen")
        conveyor1.move_conveyor_till_drop()
        time.sleep(2)
        strahlkabine.strahlen()

    def perform_action_series7(self):
        print("no PICK + PLACE 1, move_conveyor_till_drop > strahlen")
        conveyor1.move_conveyor_till_drop()
        time.sleep(2)
        strahlkabine.strahlen()

    def perform_action_series8(self):
        print("no PICK + PLACE 1, move_conveyor_till_drop > strahlen")
        conveyor1.move_conveyor_till_drop()
        if manual_mode:
            override=input("Proceed with Strahlen? y/n \n")
            if override:
                strahlkabine.strahlen()

    def perform_action_series(self, strahlen_end,conveyor1sensor1,conveyor1sensor2):
        action=self.action_map.get((strahlen_end,conveyor1sensor1,conveyor1sensor2))
        if action:
            with self.lock:
                if self.current_action_thread is None or not self.current_action_thread.is_alive():
                    self.current_action_thread = threading.Thread(target=action)
                    self.current_action_thread.start()
                else:
                    print("Action series already running. Waiting for current action to complete.")
        else:
            print(f"No action defined for states: strahlen_end:{strahlen_end}, c1s1:{conveyor1sensor1}, c1s2:{conveyor1sensor2}")

class BlockOneStateObserver:
    def __init__(self,  action_handler, strahlkabine, conveyor1sensor1, conveyor1sensor2, interval=0.1):
        self.check_interval=interval
        self.action_handler=action_handler
        self.strahlkabine=strahlkabine
        self.c1s1=conveyor1sensor1
        self.c1s2=conveyor1sensor2
        self.running=False
        self.observers=[]

        self.strahlkabine_end=False # Strahlkabine did not finish
        self.conveyor1sensor1=False # No part present
        self.conveyor1sensor2=False # No part present

        self._lock = threading.Lock() # only one thread can access a block of code or a resource at a time
    
    def read_state_and_sensors(self):
        with self._lock:
            self._strahlkabine_end=self.strahlkabine.get_strahlen_end_status()
            self._c1s1_state= self.c1s1.get_sensor_bool()
            self._c1s2_state= self.c1s2.get_sensor_bool()
            return self._strahlkabine_end, self._c1s1_state, self._c1s2_state
    
    def get_state_and_sensors(self):
        with self._lock:
            return self._strahlkabine_end, self._c1s1_state, self._c1s2_state
 
    def register_observer(self, observer):
        if observer not in self.observers:
            self.observers.append(observer)

    def notify_observers(self,strahlkabine_end, c1s1_state, c1s2_state):
        for observer in self.observers:
            observer.update(strahlkabine_end, c1s1_state, c1s2_state)
    
    def start(self):
        self.running=True
        self._thread=threading.Thread(target=self._observe)
        self._thread.daemon=True
        self._thread.start()

    def stop(self):
        self.running=False
        self._thread.join()

    def _observe(self):
        previous_states=None
        while self.running:
            strahlen_end, c1s1_state, c1s2_state =self.read_state_and_sensors()
            if (strahlen_end, c1s1_state, c1s2_state) != previous_states:
                self.notify_observers(strahlen_end, c1s1_state, c1s2_state)
                previous_states=(strahlen_end, c1s1_state, c1s2_state)
                threading.Thread(target=self.action_handler.perform_action_series, args=(strahlen_end, c1s1_state, c1s2_state)).start()
            time.sleep(self.check_interval)

if __name__ == "__main__":



    # Initialization process:
    vibratingplate, conveyor1_sensor1, conveyor1_sensor2, conveyor1, strahlkabine, ur5e, depth_cam, inspection_cam = initialize_autoclean()
    strahlen_end=True
    
    """  
    
                                BlockZero:
     Vibration process starts

     Vibration lasts for t minutes (back and forth)

                                BlockOne:

     After t minutes, bin_picking is carried out:
     1) run yolo detection
     2) run edge_depth
     3) run transformation to get TCP (AND also height of object) 

     If sensor1: LOW and sensor2: LOW:
     4) run PICK + PLACE 1 (vibrating plate to conveyor1) -> triggers sensor1: HIGH. 

     If sensor1: HIGH and sensor2: LOW:
     5) if Strahlkabine_end: HIGH, part can be dropped for cleaning
     6) if Strahlkabine_end: LOW, part cannot be dropped, if sensor2: LOW, shift part by 10cm -> sensor1: LOW
     7) if sensor2: HIGH, conveyor1 is almost full. Cannot shift part by 10cm. Loop back to (5)
     If sensor1: HIGH and sensor2: HIGH:
     8) no more parts can be placed on conveyor1. Loop back to (5)

     Strahlkabine Logic:
     strahl_end c1s1 c1s2   |                   actions
        0       0       0   |   PICK + PLACE 1, no move_conveyor, no strahlen
        0       0       1   |   PICK + PLACE 1, no move_conveyor, no strahlen
        0       1       0   |   no PICK + PLACE 1, simple_move, no strahlen
        0       1       1   |   no PICK + PLACE 1, no move_conveyor, no strahlen
        1       0       0   |   PICK + PLACE 1, no move_conveyor, no strahlen 
        1       0       1   |   PICK + PLACE 1, move_conveyor_till_drop > strahlen
        1       1       0   |   no PICK + PLACE 1, move_conveyor_till_drop > strahlen
        1       1       1   |   no PICK + PLACE 1, move_conveyor_till_drop > strahlen
        
                                BlockTwo:           
    Sorting: (4-5 Boxes)
    Versandt, Ausschuss, Nacharbeit (multiple)
    """