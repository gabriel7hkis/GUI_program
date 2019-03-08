import sys
import time
sys.path.append('/home/genia/projects/labcodes_branch')
from fluidics.cLLD_TEST_JOHN import gantryTecan_chip
from Tkinter import *

gtc = gantryTecan_chip()
window = Tk()

window_width = 1000
window_height = 800
path1 = "FMRobotDirections.gif"
path2 = "abort.gif"
instruction_str = (""" Instructions:
 1. Click Connect robot.
 2. Click Initialize robot.
 3. Some other stuff.""")
 
Syringesize = 1000
maxPullvol = 1000
maxPullspeed = 800
maxTubePull = 2000

# robot direction image
img1 = PhotoImage(file=path1)      
pic1 = Label(window, image=img1, bg="white", borderwidth=0)
pic1.place(x=0, y=0)

# abort button image
img2 = PhotoImage(file=path2)      
pic2 = Label(window, image=img2, bg="white", borderwidth=0)
pic2.place(x=450, y=575)

window.geometry("{}x{}".format(window_width, window_height))   # set window resolution
window.title("FM Robot Control Interface")
window.configure(bg="white")

# label for mouse cursor position
lbl1 = Label(window, text=" 0, 0", font=("Arial", 12), bg="white")
lbl1.place(x=0, y=0)        # COMMENT THIS OUT AFTER TESTING IS COMPLETE

# label for status
status_lbl = Label(window, text=" Status: ", font=("Arial", 10),
bg="white", borderwidth=2, relief="solid", anchor=NW)

status_lbl.place(x=window_width-360, y=window_height-55, width=350, height=50)




# group for motion control function buttons
# includes robot init, move relative, enable/disable axes
motion_cntrl_grp = Label(window, text=" Motion Control:", font=("Arial Bold", 10),
bg="white", borderwidth=2, relief="solid", anchor=NW)

motion_cntrl_grp.place(x=675, y=10, width=150, height=320)

# button for robot connect function
# connects to Tecan robot
def robot_connect_btn_click():
    status_lbl.configure(text=" Status: Connecting to robot...")
    try:
        gtc.connect()
    except:
        status_lbl.configure(text=" Status: ROBOT ALREADY CONNECTED.")
    status_lbl.configure(text=" Status: Successfully connected to robot.")
robot_connect_btn = Button(window, text="Connect robot", command=robot_connect_btn_click)
robot_connect_btn.place(in_=motion_cntrl_grp, x=5, y=20)

# button for robot disconnect function
# disconnectes from Tecan robot
def robot_disconnect_btn_click():
    status_lbl.configure(text=" Status: Disconnecting from robot...")
    gtc.disconnect()
    status_lbl.configure(text=" Status: Disconnected from robot.")
robot_disconnect_btn = Button(window, text="Disconnect robot", command=robot_disconnect_btn_click)
robot_disconnect_btn.place(in_=motion_cntrl_grp, x=5, y=50)

# button for robot initialization function
# initializes all 4 axes in the following order: z1, z2, x, y
def init_robot_btn_click():
    status_lbl.configure(text=" Status: Initializing robot...")
    gtc.init()
    status_lbl.configure(text=" Status: Robot successfully initialized.")
init_robot_btn = Button(window, text="Initialize robot", command=init_robot_btn_click)
init_robot_btn.place(in_=motion_cntrl_grp, x=5, y=80)

# button for robot enable function
# enables current to all four axes.
def enable_btn_click():
    status_lbl.configure(text=" Status: Enabling robot...")
    gtc.tecanCommObj.sendCmd('1/1N1/2N1/3N1/7N1R')
    status_lbl.configure(text=" Status: Robot successfully enabled.")
enable_btn = Button(window, text="Enable robot", command=enable_btn_click)
enable_btn.place(in_=motion_cntrl_grp, x=5, y=110)

# button for robot disable function
# enables current to all four axes.
def disable_btn_click():
    status_lbl.configure(text=" Status: Disabling robot...")
    gtc.tecanCommObj.sendCmd('1/1N0/2N0/3N0/7N0R')
    status_lbl.configure(text=" Status: Robot successfully disabled.")
disable_btn = Button(window, text="Disable robot", command=disable_btn_click)
disable_btn.place(in_=motion_cntrl_grp, x=5, y=140)

# textbox for distance in mm
lbl = Label(window, text="Distance (mm)", font=("Arial", 10), bg="white")
lbl.place(in_=motion_cntrl_grp, x=5, y=170)
txt_robot_dist = Entry(window, width=10, state='normal')
txt_robot_dist.insert(END, 10.0)
txt_robot_dist.place(in_=motion_cntrl_grp, x=5, y=190)

# textbox for speed in mm/sec
lbl = Label(window, text="Speed (mm/sec)", font=("Arial", 10), bg="white")
lbl.place(in_=motion_cntrl_grp, x=5, y=210)
txt_robot_spd = Entry(window,width=10, state='normal')
txt_robot_spd.insert(END, 100.0)
txt_robot_spd.place(in_=motion_cntrl_grp, x=5, y=230)

move_type_rel = IntVar()
rad1 = Radiobutton(window,text='Move relative', value=1, bg="white", variable=move_type_rel)
rad1.place(in_=motion_cntrl_grp, x=5, y=260)
rad2 = Radiobutton(window,text='Move absolute', value=0, bg="white", variable=move_type_rel)
rad2.place(in_=motion_cntrl_grp, x=5, y=285)
rad1.select()




# group for pump control function buttons
# includes pump init, valve position, pull, push, flow
pump_cntrl_grp = Label(window, text=" Pump Control:", font=("Arial Bold", 10),
bg="white", borderwidth=2, relief="solid", anchor=NW)

pump_cntrl_grp.place(x=window_width-145, y=10, width=135, height=375)

# button for pump initialization function
# intializes the pump
def pump_init_btn_click():
    status_lbl.configure(text=" Status: Initializing pump...")
    centrisInit()
    status_lbl.configure(text=" Status: Pump successfully initialized.")
pump_init_btn = Button(window, text="Initialize pump", command=pump_init_btn_click)
pump_init_btn.place(in_=pump_cntrl_grp, x=5, y=20)

# textbox for volume in ul
lbl = Label(window, text="Volume (ul)", font=("Arial", 10), bg="white")
lbl.place(in_=pump_cntrl_grp, x=5, y=55)
txt_pump_vol = Entry(window,width=10, state='normal')
txt_pump_vol.insert(END, 500.0)
txt_pump_vol.place(in_=pump_cntrl_grp, x=5, y=75)

# textbox for speed in ul/sec
lbl = Label(window, text="Speed (ul/sec)", font=("Arial", 10), bg="white")
lbl.place(in_=pump_cntrl_grp, x=5, y=95)
txt_pump_spd = Entry(window,width=10, state='normal')
txt_pump_spd.insert(END, 500.0)
txt_pump_spd.place(in_=pump_cntrl_grp, x=5, y=115)

# textbox for repetitions
lbl = Label(window, text="Repetitions", font=("Arial", 10), bg="white")
lbl.place(in_=pump_cntrl_grp, x=5, y=145)
txt_pump_reps = Entry(window,width=10, state='normal')
txt_pump_reps.insert(END, 1)
txt_pump_reps.place(in_=pump_cntrl_grp, x=5, y=165)

# button for pump flow function
# aspirates then dispenses the specified volume (ul) at the specified rate (ul/sec) using the Tecan Centris pump
def pump_flow_btn_click():
    vol, spd, reps = float(txt_pump_vol.get()), float(txt_pump_spd.get()), int(txt_pump_reps.get())
    status_lbl.configure(text=" Status: Flowing {}ul at {}ul/sec, {} times...".format(vol, spd, reps))
    for i in range(reps):
        centrisAuto_flow(vol, spd, inlet_valve='outlet_pos')
    status_lbl.configure(text=" Status: Flow(s) complete.")
pump_flow_btn = Button(window, text="Pump flow", command=pump_flow_btn_click)
pump_flow_btn.place(in_=pump_cntrl_grp, x=5, y=195)

# button for pump pull function
# aspirates the specified volume (ul) at the specified rate (ul/sec) using the Tecan Centris pump
def pump_pull_btn_click():
    vol, spd = float(txt_pump_vol.get()), float(txt_pump_spd.get())
    status_lbl.configure(text=" Status: Aspirating {}ul at {}ul/sec...".format(vol, spd))
    centrisPull(volume, speed, delay=0.5)
    status_lbl.configure(text=" Status: Asiprate complete.")
pump_pull_btn = Button(window, text="Pump pull", command=pump_pull_btn_click)
pump_pull_btn.place(in_=pump_cntrl_grp, x=5, y=225)

# button for pump push function
# dispenses the specified volume (ul) at the specified rate (ul/sec) using the Tecan Centris pump
def pump_push_btn_click():
    vol, spd = float(txt_pump_vol.get()), float(txt_pump_spd.get())
    status_lbl.configure(text=" Status: Dispensing {}ul at {}ul/sec...".format(vol, spd))
    centrisPush(vol, spd, delay=0.5)
    status_lbl.configure(text=" Status: Dispense complete.")
pump_push_btn = Button(window, text="Pump push", command=pump_push_btn_click)
pump_push_btn.place(in_=pump_cntrl_grp, x=5, y=255)

# textbox for valve position
lbl = Label(window, text="Valve position", font=("Arial", 10), bg="white")
lbl.place(in_=pump_cntrl_grp, x=5, y=285)
txt_pump_valve_pos = Entry(window,width=10, state='normal')
txt_pump_valve_pos.insert(END, 1)
txt_pump_valve_pos.place(in_=pump_cntrl_grp, x=5, y=305)

# button for pump valve function
# moves to the specified pump valve position
def pump_valve_btn_click():
    valve_pos = txt_pump_valve_pos.get()
    status_lbl.configure(text=" Status: Changing pump valve position to {}...".format(valve_pos))
    centrisValvePos(valve_pos)
    status_lbl.configure(text=" Status: Pump valve change complete.")
pump_valve_btn = Button(window, text="Pump valve", command=pump_valve_btn_click)
pump_valve_btn.place(in_=pump_cntrl_grp, x=5, y=335)




# group for miscellaneous function buttons
# includes force pierce, clld detect, antisiphon valve control, waste pump control, send command
misc_cntrl_grp = Label(window, text=" Misc Functions:", font=("Arial Bold", 10),
bg="white", borderwidth=2, relief="solid", anchor=NW)

misc_cntrl_grp.place(x=window_width-325, y=400, width=315, height=335)

# textbox for axis
lbl = Label(window, text="Pierce axis", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=5, y=15)
txt_axs = Entry(window, width=10, state='normal')
txt_axs.insert(END, 'Z1')
txt_axs.place(in_=misc_cntrl_grp, x=5, y=35)

# textbox for force value
lbl = Label(window, text="Force (lbf)", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=5, y=60)
txt_force = Entry(window,width=10, state='normal')
txt_force.insert(END, 1.0)
txt_force.place(in_=misc_cntrl_grp, x=5, y=80)

# button for force pierce function
# applies specified force in lbf using specified axis
def force_pierce_btn_click():
    force, axs = float(txt_force.get()), txt_axs.get().lower()
    status_lbl.configure(text=" Status: Applying {}lbf using axis {}...".format(force, axs))
    gtc.forcePierce(axs, force)
    status_lbl.configure(text=" Status: Force pierce complete.")
force_pierce_btn = Button(window, text="Force pierce", command=force_pierce_btn_click)
force_pierce_btn.place(in_=misc_cntrl_grp, x=5, y=110)

# textbox for cLLD search direction
lbl = Label(window, text="cLLD search direction", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=5, y=155)
txt_cLLD_search_dir = Entry(window,width=10, state='normal')
txt_cLLD_search_dir.insert(END, 'X')
txt_cLLD_search_dir.place(in_=misc_cntrl_grp, x=5, y=175)

# textbox for cLLD search axis
lbl = Label(window, text="cLLD search axis", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=5, y=200)
txt_cLLD_search_axis = Entry(window,width=10, state='normal')
txt_cLLD_search_axis.insert(END, "Z2")
txt_cLLD_search_axis.place(in_=misc_cntrl_grp, x=5, y=220)

# textbox for cLLD search range
lbl = Label(window, text="cLLD search range (mm)", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=5, y=245)
txt_cLLD_search_range = Entry(window,width=10, state='normal')
txt_cLLD_search_range.insert(END, 10)
txt_cLLD_search_range.place(in_=misc_cntrl_grp, x=5, y=265)

# button for cLLD search function
# uses Tecan cLLD function to search for conductive target in specified direction, using specified axis and range
def cLLD_search_btn_click():
    search_axs, search_dir, search_range = txt_cLLD_search_axis.get(), txt_cLLD_search_dir.get(), txt_cLLD_search_range.get()
    status_lbl.configure(text=" Status: Using {} axis to search in {} direction over {} range...".format(search_axs, search_dir, search_range))
    time.sleep(2)
    status_lbl.configure(text=" Status: cLLD search complete.")
cLLD_search_btn = Button(window, text="cLLD search", command=cLLD_search_btn_click)
cLLD_search_btn.place(in_=misc_cntrl_grp, x=5, y=295)

# button for waste pump on function
# turns on waste pump
def waste_pump_on_btn_click():
    robot_cmd = txt_robot_cmd.get()
    status_lbl.configure(text=" Status: Turning waste pump on...")
    centrisWastePumpOn()
    status_lbl.configure(text=" Status: Waste pump turned on.")
waste_pump_on_btn = Button(window, text="Waste pump on", command=waste_pump_on_btn_click)
waste_pump_on_btn.place(in_=misc_cntrl_grp, x=160, y=5)

# button for waste pump off function
# turns off waste pump
def waste_pump_off_btn_click():
    robot_cmd = txt_robot_cmd.get()
    status_lbl.configure(text=" Status: Turning waste pump off...")
    centrisWastePumpoff()
    status_lbl.configure(text=" Status: Waste pump turned off.")
waste_pump_off_btn = Button(window, text="Waste pump off", command=waste_pump_off_btn_click)
waste_pump_off_btn.place(in_=misc_cntrl_grp, x=160, y=35)

# antisiphon valve control functions
lbl = Label(window, text="Antisiphon valve control", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=125, y=65)

# antisiphon valve 1 open button
# opens antisiphon valve 1
def asvalve1_open_btn_click():
    status_lbl.configure(text=" Status: Opening A/S valve 1...")
    centrisOpenPinchValve()
    status_lbl.configure(text=" Status: A/S valve 1 open.")
asvalve1_open_btn = Button(window, text="1: open", command=asvalve1_open_btn_click)
asvalve1_open_btn.place(in_=misc_cntrl_grp, x=150, y=85)

# antisiphon valve 1 close button
# closes antisiphon valve 1
def asvalve1_close_btn_click():
    status_lbl.configure(text=" Status: Closing A/S valve 1...")
    centrisClosePinchValve()
    status_lbl.configure(text=" Status: A/S valve 1 closed.")
asvalve1_close_btn = Button(window, text="1: close", command=asvalve1_close_btn_click)
asvalve1_close_btn.place(in_=misc_cntrl_grp, x=225, y=85)

# antisiphon valve 2 open button
# opens antisiphon valve 2
def asvalve2_open_btn_click():
    status_lbl.configure(text=" Status: Opening A/S valve 2...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 2 open.")
asvalve2_open_btn = Button(window, text="2: open", command=asvalve2_open_btn_click)
asvalve2_open_btn.place(in_=misc_cntrl_grp, x=150, y=115)

# antisiphon valve 2 close button
# closes antisiphon valve 2
def asvalve2_close_btn_click():
    status_lbl.configure(text=" Status: Closing A/S valve 2...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 2 closed.")
asvalve2_close_btn = Button(window, text="2: close", command=asvalve2_close_btn_click)
asvalve2_close_btn.place(in_=misc_cntrl_grp, x=225, y=115)

# antisiphon valve 3 open button
# opens antisiphon valve 3
def asvalve3_open_btn_click():
    status_lbl.configure(text=" Status: Opening A/S valve 3...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 3 open.")
asvalve3_open_btn = Button(window, text="3: open", command=asvalve3_open_btn_click)
asvalve3_open_btn.place(in_=misc_cntrl_grp, x=150, y=145)

# antisiphon valve 3 close button
# closes antisiphon valve 3
def asvalve3_close_btn_click():
    status_lbl.configure(text=" Status: Closing A/S valve 3...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 3 closed.")
asvalve3_close_btn = Button(window, text="3: close", command=asvalve3_close_btn_click)
asvalve3_close_btn.place(in_=misc_cntrl_grp, x=225, y=145)

# antisiphon valve 4 open button
# opens antisiphon valve 4
def asvalve4_open_btn_click():
    status_lbl.configure(text=" Status: Opening A/S valve 4...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 4 open.")
asvalve4_open_btn = Button(window, text="4: open", command=asvalve4_open_btn_click)
asvalve4_open_btn.place(in_=misc_cntrl_grp, x=150, y=175)

# antisiphon valve 4 close button
# closes antisiphon valve 4
def asvalve4_close_btn_click():
    status_lbl.configure(text=" Status: Closing A/S valve 4...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 4 closed.")
asvalve4_close_btn = Button(window, text="4: close", command=asvalve4_close_btn_click)
asvalve4_close_btn.place(in_=misc_cntrl_grp, x=225, y=175)

# antisiphon valve 5 open button
# opens antisiphon valve 5
def asvalve5_open_btn_click():
    status_lbl.configure(text=" Status: Opening A/S valve 5...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 5 open.")
asvalve5_open_btn = Button(window, text="5: open", command=asvalve5_open_btn_click)
asvalve5_open_btn.place(in_=misc_cntrl_grp, x=150, y=205)

# antisiphon valve 5 close button
# closes antisiphon valve 5
def asvalve5_close_btn_click():
    status_lbl.configure(text=" Status: Closing A/S valve 5...")
    time.sleep(2)
    status_lbl.configure(text=" Status: A/S valve 5 closed.")
asvalve5_close_btn = Button(window, text="5: close", command=asvalve5_close_btn_click)
asvalve5_close_btn.place(in_=misc_cntrl_grp, x=225, y=205)

# textbox for Tecan command
lbl = Label(window, text="Tecan command:", font=("Arial", 10), bg="white")
lbl.place(in_=misc_cntrl_grp, x=165, y=245)
txt_robot_cmd = Entry(window,width=15, state='normal')
txt_robot_cmd.insert(END, '1/3ZR')
txt_robot_cmd.place(in_=misc_cntrl_grp, x=165, y=265)

# button for Tecan send command
# sends command to the Tecan robot
def robot_send_cmd_btn_click():
    robot_cmd = txt_robot_cmd.get()
    status_lbl.configure(text=" Status: Sending cmd: {}...".format(robot_cmd))
    gtc.tecanCommObj.sendCmd(robot_cmd)
    status_lbl.configure(text=" Status: Command {} sent.".format(robot_cmd))
robot_send_cmd_btn = Button(window, text="Send cmd", command=robot_send_cmd_btn_click)
robot_send_cmd_btn.place(in_=misc_cntrl_grp, x=185, y=295)





# label for instructions
lbl = Label(window, text=" Instructions:", font=("Arial Bold", 12),
bg="white", borderwidth=2, relief="solid", justify="left", anchor=NW)

lbl.place(x=10, y=window_height-240, width=400, height=235)
lbl.configure(text=(instruction_str))

# robot direction buttons
def click1(event):
    x, y = event.x, event.y
    lbl1.configure(text=' clicked at {}, {}'.format(x,y))       # COMMENT THIS OUT AFTER TESTING IS COMPLETE
    dist, spd, rel = float(txt_robot_dist.get()), float(txt_robot_spd.get()), move_type_rel.get()
    
    if 420<=x<=470 and 235<=y<=285:     # +X button is clicked
        clicked_axis = "X"
        clicked_dir = 1
    elif 35<=x<=85 and 205<=y<=260:       # -X button is clicked
        clicked_axis = "X"
        clicked_dir = -1
    elif 95<=x<=150 and 320<=y<=370:      # +Y button is clicked
        clicked_axis = "Y"
        clicked_dir = 1
    elif 345<=x<=395 and 100<=y<=160:     # -Y button is clicked
        clicked_axis = "Y"
        clicked_dir = -1
    elif 290<=x<=365 and 35<=y<=85:       # +Z1 button is clicked
        clicked_axis = "Z1"
        clicked_dir = -1
    elif 290<=x<=355 and 420<=y<=465:     # -Z1 button is clicked
        clicked_axis = "Z1"
        clicked_dir = 1
    elif 145<=x<=225 and 65<=y<=120:      # +Z2 button is clicked
        clicked_axis = "Z2"
        clicked_dir = -1
    elif 160<=x<=230 and 230<=y<=430:     # -Z2 button is clicked
        clicked_axis = "Z2"
        clicked_dir = 1
    else:
        return 0    
    status_lbl.configure(text=" Status: Moving {} axis {}mm at {}mm/sec...".format(clicked_axis, dist, spd))
    if rel:
        gtc.move(clicked_axis, clicked_dir*dist, spd)
    elif not rel:
        gtc.moveABS(clicked_axis, dist, spd)
    status_lbl.configure(text=" Status: Moved {} axis {}mm at {}mm/sec.".format(clicked_axis, dist, spd))

# abort button      
def click2(event):
    x, y = event.x, event.y
    if 0<=x<=175 and 0<=y<=175:     # abort button is clicked
        gtc.tecanCommObj._sendNonBlockingCmd('0/T')
        status_lbl.configure(text=" Status: Aborted!")
        gtc.tecanCommObj._recvReply()
    
#window.bind("<Button-1>", click1)       # COMMENT THIS OUT AFTER TESTING IS COMPLETE
pic1.bind("<Button-1>", click1)
pic2.bind("<Button-1>",click2)

def centrisInit():
    gtc.tecanCommObj.sendCmd('1/9Z7,1,1R')
    gtc.tecanCommObj.sendCmd('1/9U11R')

def centrisSetSpeed(speed=maxPullspeed):
    gtc.tecanCommObj.sendCmd('1/9V{},1'.format(str(speed)))

def centrisValvePos(spot):
    spot = spot.lower()
    if spot == 'a' or spot == 'inlet_pos':
        spot = 1
    elif spot == 'b' or spot == 'outlet_pos':
        spot = 2
    elif spot == 'c' or spot == 'waste_pos':
        spot = 3
    elif spot == 'd' or spot == 'outlet_pos_dis':
        spot = 4
    elif spot == 'e':
        spot = 5
    elif spot == 'f':
        spot = 6
    gtc.tecanCommObj.sendCmd('1/9I{}R'.format(str(spot)))

def centrisPull(volume, speed=maxPullspeed, delay=0.5):
    if volume > 1000:
        print('Pull volume is too large defaulting to the max pull volume of {}ul'.format(str(maxPullvol)))
        volume = 1000
    print('Pulling {}ul at {}ul/sec'.format(str(volume),str(speed)))
    centrisSetSpeed(speed)
    #step_name, flag = ps.start_sensor()
    #t1 = threading.Thread(target=ps.read_and_record, args=(step_name, flag, "pump_pull"));t1.start()
    gtc.tecanCommObj.sendCmd('1/9P{},1R'.format(str(volume)))
    time.sleep(delay)
    #ps.end_sensor(step_name)
    #t1.join()

def centrisPush(volume, speed=maxPullspeed, delay=0.0):
    print('Pushing {}ul at {}ul/sec'.format(str(volume),str(speed)))
    centrisSetSpeed(speed)
    centrisOpenPinchValve()
    #step_name, flag = ps.start_sensor()
    #t1 = threading.Thread(target=ps.read_and_record, args=(step_name, flag, "pump_push"));t1.start()
    gtc.tecanCommObj.sendCmd('1/9D{},1R'.format(str(volume)))
    time.sleep(delay)
    #ps.end_sensor(step_name)
    #t1.join()
    centrisClosePinchValve()

def centrisClosedPush(volume, speed=maxPullspeed, delay=0.0):
    print('Pushing {}ul at {}ul/sec'.format(str(volume),str(speed)))
    centrisSetSpeed(speed)
    centrisClosePinchValve()
    #step_name, flag = ps.start_sensor()
    #t1 = threading.Thread(target=ps.read_and_record, args=(step_name, flag, "pump_push"));t1.start()
    gtc.tecanCommObj.sendCmd('1/9D{},1R'.format(str(volume)))
    time.sleep(delay)
    #ps.end_sensor(step_name)
    #t1.join()

def centrisOpenPinchValve():
    print('Opening Anti-Siphon valve')
    #step_name, flag = ps.start_sensor()
    #t1 = threading.Thread(target=ps.read_and_record, args=(step_name, flag, "openPinchValve"));t1.start()
    gtc.tecanCommObj.sendCmd('1/0L1,1R')
    #ps.end_sensor(step_name)
    #t1.join()

def centrisClosePinchValve():
    print('Closing Anti-Siphon valve')
    #step_name, flag = ps.start_sensor()
    #t1 = threading.Thread(target=ps.read_and_record, args=(step_name, flag, "closePinchValve"));t1.start()
    gtc.tecanCommObj.sendCmd('1/0L1,0R')
    #ps.end_sensor(step_name)
    #t1.join()

def centrisWastePumpOn():
    print('Turning the waste pump on')
    gtc.tecanCommObj.sendCmd('1/0L2,1R')

def centrisWastePumpoff():
    print('Turning the waste pump off')
    gtc.tecanCommObj.sendCmd('1/0L2,0R')

def centrisZeroInlet(speed=maxPullspeed, delay=1):
    centrisSetSpeed(speed)
    centrisValvePos('inlet_pos')
    print('Zeroing pump to inlet')
    gtc.tecanCommObj.sendCmd('1/9A0,1')
    time.sleep(delay)

def centrisAuto_flow(volume, speed, inlet_valve='inlet_pos', outlet_valve='outlet_pos', delay=0.0):
    centrisValvePos('inlet_pos')
    centrisZeroInlet()
    centrisValvePos(inlet_valve)
    centrisPull(volume, maxPullspeed)
    centrisValvePos(outlet_valve)
    centrisPush(volume, speed, delay=delay)

def centrisAuto_closedflow(volume, speed, inlet_valve='inlet_pos',
                           outlet_valve='outlet_pos', delay=0.0):
    centrisValvePos('inlet_pos')
    centrisZeroInlet()
    centrisClosePinchValve()
    centrisValvePos(inlet_valve)
    centrisPull(volume, maxPullspeed)
    centrisValvePos(outlet_valve)
    centrisClosedPush(volume, speed, delay=delay)

mainloop()
