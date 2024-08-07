from tools.ur_realtime import URRealTime
import threading, time, queue
from check_calibration import InspectionCameraApp, DepthCameraApp
from transformation import RobotControl, ObjectGrasp
from tools.processes import save_instance_to_json, load_camera_instance_from_json
from edge_depth import PcdData, Detection
from inspection.object_recognition_V1 import object_recognition
from inspection.powder_recognition_V3 import powder_recognition

inspection=True
pickvibratingplate=False
initialize_depth_camera=False
grasp_for_inspection=True
inspection_complete=False

test_target=True
grasp_target=None
grasping_possible=True

# action numbers:
pick_part_vibrate_plate=3
part_inspection=0
part_conveyor1_drop=1
part_sorting=2



if __name__ == "__main__":
    ur_autoclean = URRealTime()
    ur5e=ur_autoclean
    camera=InspectionCameraApp()
    depthcamera=DepthCameraApp()
    if ur_autoclean.ur_ready():
        ur_autoclean.ur_poses_initiliaze()
        while False:
            ur_autoclean.ur_save_pose()         
        
        if initialize_depth_camera:
            start_pose=ur_autoclean.ur_checklog()
            end_pose = 3
            ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)
            confirmrobotposition="n"
            while confirmrobotposition != "y":
                confirmrobotposition=input("Is robot at required vibrating plate position to initialize camera? (y/n)")
                time.sleep(0.1)
            confirmrobotposition="n"
            if depthcamera.initialize_camera(): # nominal depth to be used later for conveyor drop
                save_instance_to_json(depthcamera)
                time.sleep(0.1)
                grasping_possible=True
            else:
                grasping_possible=False      
        
        if pickvibratingplate and test_target and not initialize_depth_camera:
            start_pose=ur_autoclean.ur_checklog()
            end_pose = 3
            ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)

            ur_autoclean.ur_perform_action(pick_part_vibrate_plate, 
                                           grasp_target=grasp_target,
                                           test_target=True)

            start_pose=ur_autoclean.ur_checklog()
            end_pose=5
            ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)
            ur_autoclean.ur_perform_action(part_conveyor1_drop)
             
        if pickvibratingplate and grasping_possible and not initialize_depth_camera:
            start_pose=ur_autoclean.ur_checklog()
            end_pose = 3
            ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)
            confirmrobotposition="n"
            while confirmrobotposition != "y":
                confirmrobotposition=input("Is robot at required vibrating plate position to capture image? (y/n)")
                time.sleep(0.1)
            confirmrobotposition="n"
            
            depthcamera.capture_image()
            depthcamera.capture_pcd()
            depth_camera =  load_camera_instance_from_json()
            pcd=PcdData()
            detection=Detection(depth_camera, pcd)
            for graspable_edge in detection.graspable_edges:
                print(graspable_edge.edge_grasp_frame)
                graspable_edge.show()

            confirm_grasp="n"
            while confirm_grasp != "y":
                confirm_grasp=input("Object grasped? (y/n)")
                time.sleep(0.1)
                if confirm_grasp=="abort":
                    break
            
            if confirm_grasp=="abort":
                print("Grasping sequence terminated.")
            else:
                ur_autoclean.ur_perform_action(pick_part_vibrate_plate, 
                                            grasp_target=grasp_target,
                                            test_target=test_target)

                start_pose=ur_autoclean.ur_checklog()
                end_pose=5
                ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)
                ur_autoclean.ur_perform_action(part_conveyor1_drop)
        
        def excel_access(avg_class, powder_state):
            pass

        def ur_inspection():
            ur_autoclean.ur_perform_action(part_inspection)
        
        def cam_inspection():
            if camera.capture_window(data_type='image', clear_all_images=True):
                interval=5 # s
                #camera.display_webcam_feed()
                # Capture images and keep latest N images in folder
                max_images=40
            
                while True:
                    number_of_images=camera.capture_image(max_images)
                    if number_of_images>=max_images:
                        break
                    #print(f"Latest image captured is : {camera.capture_image(20)}")

                    time.sleep(interval)
                
        def object_and_powder_recognition():
            avg_class = object_recognition() 
            powder_state = powder_recognition() # 'Clean' or 'Powdered'
            if powder_state=='Clean':
                decision = excel_access(avg_class, powder_state) # 'Loop' or 'Nacharbeit1 - 4'
                decision='a'
            elif powder_state=='Powdered':
                decision='Loop'
            else:
                print("No decision.")
            return decision

        if inspection and not initialize_depth_camera and grasp_for_inspection and not inspection_complete:
            start_pose=ur_autoclean.ur_checklog()
            end_pose = 0
            ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)
            ur_autoclean.ur_perform_action(pick_part_vibrate_plate, test_target=True)

            start_pose=ur_autoclean.ur_checklog()
            end_pose=4
            ur_autoclean.ur_move_from_A_to_B(start_pose,end_pose)
            # Create and start threads for inspection motion and running camera in parallel:
            thread_ur = threading.Thread(target=ur_inspection)
            thread_camera = threading.Thread(target=cam_inspection)
                    
            thread_ur.start()
            time.sleep(20)
            thread_camera.start()

            # Wait for threads to complete
            thread_ur.join()
            thread_camera.join()

            print("Inspection completed.")
            inspection_complete=True
        
        if inspection_complete:
            state=object_and_powder_recognition()
                    
                # Conditions after inspection:
        
        #state='a' # 'b', 'Loop' # a is box a, b is box b, c is clean again

        if state=='a':
            start_pose=ur5e.ur_checklog()
            end_pose =1
            ur5e.ur_move_from_A_to_B(start_pose,end_pose)
            ur5e.ur_perform_action(part_sorting, sort=0, test_target=True)
        elif state=='b':
            start_pose=ur5e.ur_checklog()
            end_pose =1
            ur5e.ur_move_from_A_to_B(start_pose,end_pose)
            ur5e.ur_perform_action(part_sorting, sort=1,test_target=True)
        elif state=='Loop':
            #if not conveyor1_sensor1.get_sensor_bool:
            start_pose=ur5e.ur_checklog()
            end_pose =5
            ur5e.ur_move_from_A_to_B(start_pose,end_pose)
            ur5e.ur_perform_action(part_conveyor1_drop)