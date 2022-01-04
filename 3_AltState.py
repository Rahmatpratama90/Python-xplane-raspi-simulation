# -*- coding: utf-8 -*-
"""
Created on Sat Jul 31 20:36:51 2021

@author: User
"""

from time import sleep
import xpc
import matplotlib  as plt

#Calculate PID        
def PID(xd, x, Kp, Ki, Kd, e_acc, e_old):
    e = xd - x
    p = e
    i = (e + e_acc)*0.05
    d = (e - e_old)/0.05
    u = Kp*p + Ki*i + Kd*d
    e_acc = e_acc + e
    e_old = e
    value = [u, e_acc, e_old]
    
    return value

with xpc.XPlaneConnect() as client:
    pitch  = [0, 0, 0]
    elev = [0, 0, 0]
    roll = [0, 0, 0]
    ailr = [0, 0, 0]
    listtime = []
    listalt = []
    listVVI = []
    listGload = []
    listPitch = []
    listYaw = []
    print("Getting Position")
    result = client.readDATA();
    lat = result[5][1]
    lon = result[5][2]
    alt = result[5][6]
    print("lat =",lat,"lon =",lon,"alt =",alt)
    
    #Set Gear and Brake
    data = [[14, 1, 0, -999, -999, 0, -999, -999, -999]]
    client.sendDATA(data)
    
    #Set Throtle
    data = [[25, 1, -999, -999, -999, -999, -999, -999, -999]]
    client.sendDATA(data)
    
    sleep(5)
    
    #State 1
    print("state 1")
    print("Starting Time = ",result[0][3])    

    while alt < 500:
        
        #Heading (change set point)
        result = client.readDATA();
        alt = result[5][6]
        listtime.append(result[0][3])
        listVVI.append(result[1][3])
        listGload.append(result[1][5])
        listPitch.append(result[4][1])
        heading = result[4][4]
        listYaw.append(heading)
        
        yawtarget = 114.88
        target = yawtarget - heading
        if target < 180:
            yawtarget = yawtarget
            
        elif target > 180:
            yawtarget = yawtarget - 360
        
        roll = PID(yawtarget, heading, 0.05, 0.02, 0.002, 0, 0)
        if roll[0] > 45:
            roll[0] = 45
        elif roll[0] < -45:
            roll[0] = -45
        
        #Roll
        rollnow = result[4][2]
    
        ailr = PID(roll[0], rollnow, 0.05, 0.02, 0.002, 0, 0)
        if ailr[0] > 1:
            ailr[0] = 1
        elif ailr[0] < -1:
            ailr[0] = -1    
        
        
        #Alt (change set point)
        altnow = result[5][6]
        listalt.append(result[5][6])
        
        pitch = PID(2000, altnow, 0.5, 0.002, 0.0002, 0, 0)
        if pitch[0] > 30:
            pitch[0] = 30
        elif pitch[0] < -30:
            pitch[0] = -30       
        
        #pitch
        pitchnow = result[4][1]
        
        elev = PID(pitch[0], pitchnow, 0.05, 0.002, 0.0002, 0, 0)
        if elev[0] > 1:
            elev[0] = 1
        elif elev[0] < -1:
            elev[0] = -1
    
        data = [[17, -999, -999, -999, -999, -999, -999, -999, -999],\
                [11, elev[0], ailr[0], -999, -999, -999, -999, -999, -999]]
        client.sendDATA(data)
    
        
        continue
    
    #Set Gear and Brake
    data = [[14, 0, 0, -999, -999, 0, -999, -999, -999]]
    client.sendDATA(data)
    
    while alt < 1000:
        
        #Heading (change set point)
        result = client.readDATA();
        alt = result[5][6]
        listtime.append(result[0][3])
        listVVI.append(result[1][3])
        listGload.append(result[1][5])
        heading = result[4][4]
        listPitch.append(result[4][1])
       
        listYaw.append(heading)
        yawtarget = 300
        target = yawtarget - heading
        if target < 180:
            yawtarget = yawtarget
        elif target > 180:
            yawtarget = yawtarget - 360
        roll = PID(yawtarget, heading, 0.8, 0.02, 0.004, 0, 0) #edit ini
        #roll = PID(yawtarget, heading, 0.05, 0.02, 0.004, 0, 0) #edit ini
        if roll[0] > 45:
            roll[0] = 45
        elif roll[0] < -45:
            roll[0] = -45
        
        #Roll
        rollnow = result[4][2]
        
        ailr = PID(roll[0], rollnow, 0.05, 0.02, 0.004, 0, 0)
        if ailr[0] > 1:
            ailr[0] = 1
        elif ailr[0] < -1:
            ailr[0] = -1    
        
        
        #Alt (change set point)
        altnow = result[5][6]
        listalt.append(result[5][6])
        
        pitch = PID(1000, altnow, 0.5, 0.002, 0.0002, 0, 0)
        if pitch[0] > 30:
            pitch[0] = 30
        elif pitch[0] < -30:
            pitch[0] = -30       
        
        #pitch
        pitchnow = result[4][1]
        
        elev = PID(pitch[0], pitchnow, 0.05, 0.002, 0.0002, 0, 0)
        if elev[0] > 0.5:
            elev[0] = 0.5
        elif elev[0] < -0.5:
            elev[0] = -0.5
    
        data = [[17, -999, -999, -999, -999, -999, -999, -999, -999],\
                [11, elev[0], ailr[0], -999, -999, -999, -999, -999, -999]]
        client.sendDATA(data)
        
        continue
    
    print("Stoping Time = ",result[0][3])
    
    #State 1 Turn
    print("state 1 turn")
    print("Starting Time = ",result[0][3])
    
    while 351 <= heading <= 349 :

        
        #Heading (change set point)
        result = client.readDATA();
        listtime.append(result[0][3])
        listVVI.append(result[1][3])
        listGload.append(result[1][5])
        
        heading = result[4][4]
        yawtarget = 350 #edit ini doang
        target = yawtarget - heading
        listPitch.append(result[4][1])
       
        listYaw.append(heading)
        if target < 180:
            yawtarget = yawtarget
        elif target > 180:
            yawtarget = yawtarget - 360
       # roll = PID(yawtarget, heading, 0.20, 0.02, 0.004, 0, 0)
        roll = PID(yawtarget, heading, 0.20, 0.02, 0.1, 0, 0)
        if roll[0] > 45:
            roll[0] = 45
        elif roll[0] < -45:
            roll[0] = -45
        
        #Roll
        rollnow = result[4][2]
        
        #ailr = PID(roll[0], rollnow, 0.05, 0.02, 0.004, 0, 0)
        ailr = PID(roll[0], rollnow, 0.05, 0.02, 0.004, 0, 0)
        if ailr[0] > 1:
            ailr[0] = 1
        elif ailr[0] < -1:
            ailr[0] = -1    
        
        
        #Alt (change set point)
        #altnow = result[5][6]
        #listalt.append(result[5][6])
        
        #pitch = PID(2000, altnow, 0.5, 0.002, 0.0002, 0, 0)
        #if pitch[0] > 30:
        #    pitch[0] = 30
        #elif pitch[0] < -30:
        #    pitch[0] = -30       
        
        #pitch
        pitchnow = result[4][1]
        # elev = PID(5, pitchnow, 0.007, 0.005, 0.004, 0, 0)
        elev = PID(15, pitchnow, 0.007, 0.005, 0, 0, 0)
        if elev[0] > 1:
            elev[0] = 1
        elif elev[0] < -1:
            elev[0] = -1
    
        data = [[17, -999, -999, -999, -999, -999, -999, -999, -999],\
                [11, elev[0], ailr[0], -999, -999, -999, -999, -999, -999]]
        client.sendDATA(data)
        
        continue
    
    print("Stoping Time = ",result[0][3])
    
    #State 2    
    print("state 2")
    print("Starting Time = ",result[0][3])
    
    while alt < 2000:
        
        #Heading (change set point)
        result = client.readDATA();
        alt = result[5][6]
        listtime.append(result[0][3])
        listVVI.append(result[1][3])
        listGload.append(result[1][5])
        heading = result[4][4]
        listPitch.append(result[4][1])
       
        listYaw.append(heading)
        yawtarget = 350
        target = yawtarget - heading
        if target < 180:
            yawtarget = yawtarget
        elif target > 180:
            yawtarget = yawtarget - 360
        
      # roll = PID(yawtarget, heading, 0.05, 0.02, 0.002, 0, 0)
        roll = PID(yawtarget, heading, 0.5, 0.02, 0.002, 0, 0)
        if roll[0] > 45:
            roll[0] = 45
        elif roll[0] < -45:
            roll[0] = -45
        
        #Roll
        rollnow = result[4][2]
        
        ailr = PID(roll[0], rollnow, 0.05, 0.02, 0.002, 0, 0)
        if ailr[0] > 1:
            ailr[0] = 1
        elif ailr[0] < -1:
            ailr[0] = -1    
        
        
        #Alt (change set point)
        altnow = result[5][6]
        listalt.append(result[5][6])
        
        pitch = PID(2000, altnow, 0.5, 0.002, 0.0002, 0, 0)
        if pitch[0] > 30:
            pitch[0] = 30
        elif pitch[0] < -30:
            pitch[0] = -30       
        
        #pitch
        pitchnow = result[4][1]
        
        elev = PID(pitch[0], pitchnow, 0.05, 0.002, 0.0002, 0, 0)
        if elev[0] > 1:
            elev[0] = 1
        elif elev[0] < -1:
            elev[0] = -1
    
        data = [[17, -999, -999, -999, -999, -999, -999, -999, -999],\
                [11, elev[0], ailr[0], -999, -999, -999, -999, -999, -999]]
        client.sendDATA(data)
        
        continue
    
    print("Stoping Time = ",result[0][3])
    
    #State 3
    print("state 3")
    print("Starting Time = ",result[0][3])
    
    while alt < 3000:
        
        #Heading (change set point)
        result = client.readDATA();
        alt = result[5][6]
        listtime.append(result[0][3])
        listVVI.append(result[1][3])
        listGload.append(result[1][5])
        heading = result[4][4]
        yawtarget = 350
        target = yawtarget - heading
        listPitch.append(result[4][1])
       
        listYaw.append(heading)
        if target < 180:
            yawtarget = yawtarget
        elif target > 180:
            yawtarget = yawtarget - 360
        #roll = PID(yawtarget, heading, 0.05, 0.02, 0.002, 0, 0)
        roll = PID(yawtarget, heading, 0.7, 0.02, 0.002, 0, 0)
        if roll[0] > 45:
            roll[0] = 45
        elif roll[0] < -45:
            roll[0] = -45
        
        #Roll
        rollnow = result[4][2]
        
        ailr = PID(roll[0], rollnow, 0.05, 0.02, 0.002, 0, 0)
        if ailr[0] > 1:
            ailr[0] = 1
        elif ailr[0] < -1:
            ailr[0] = -1    
        
        
        #Alt (change set point)
        altnow = result[5][6]
        listalt.append(result[5][6])
        
        pitch = PID(3000, altnow, 0.5, 0.002, 0.0002, 0, 0)
        if pitch[0] > 30:
            pitch[0] = 30
        elif pitch[0] < -30:
            pitch[0] = -30       
        
        #pitch
        pitchnow = result[4][1]
        
        elev = PID(pitch[0], pitchnow, 0.05, 0.002, 0.0002, 0, 0)
        if elev[0] > 1:
            elev[0] = 1
        elif elev[0] < -1:
            elev[0] = -1
    
        data = [[17, -999, -999, -999, -999, -999, -999, -999, -999],\
                [11, elev[0], ailr[0], -999, -999, -999, -999, -999, -999]]
        client.sendDATA(data)
        
        continue
    
 
    plt.plot(listalt,listPitch)
    plt.ylabel("Pitch (deg)")
    plt.xlabel("Alt (feet)")
    plt.title("Grafik Pitch terhadap Altitude")
    plt.show()
    
    plt.plot(listalt,listYaw)
    plt.ylabel("Yaw (deg)")
    plt.xlabel("Alt (feet)")
    plt.title("Grafik Yaw terhadap Altitude")
    plt.show()
    
    '''
    plt.plot(listtime,listGload)
    plt.xlabel("Waktu (s)")
    plt.ylabel("Gload Normal")
    plt.title("Grafik Ketinggian terhadap waktu")
    plt.show()
    '''
    print("Stoping Time = ",result[0][3])
    
    #State 4
    print("state 4")
    
    while True:
        
        result = client.readDATA();
        rollnow = result[4][2]
        
        ailr = PID(0, rollnow, 0.05, 0.002, 0.002, 0, 0)
        if ailr[0] > 1:
            ailr[0] = 1
        elif ailr[0] < -1:
            ailr[0] = -1
        
        #pitch
        pitchnow = result[4][1]
        
        elev = PID(5, pitchnow, 0.007, 0.005, 0, 0, 0)
        if elev[0] > 1:
            elev[0] = 1
        elif elev[0] < -1:
            elev[0] = -1
    
        data = [[17, -999, -999, -999, -999, -999, -999, -999, -999],\
                [11, elev[0], ailr[0], -999, -999, -999, -999, -999, -999]]
        client.sendDATA(data)
                
            
            
