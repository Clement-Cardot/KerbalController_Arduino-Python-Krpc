# Import libraries
import time, math, serial, krpc, struct
import numpy as np
import numpy.linalg as la


arduino_connected = False
krpc_connected = False
ready = False
init = True
disconnect = True
LCDmode = 0


class KSP():

    def Handshake():
        global ready
        global arduino_connected
        global krpc_connected
        global conn
        global arduino_mega
        global vessel
        global start
        global disconnect
        while not ready :
            
            if not arduino_connected : 
                try : 
                    arduino_mega = serial.Serial('COM3', 9600,timeout=0.1)
                    time.sleep(1)
                except serial.serialutil.SerialException :
                    print('Controlleur introuvable')
                    time.sleep(.5)
                else :
                    arduino_mega.write(str.encode("#1234$"))
                    time.sleep(5)
                    while not arduino_connected :
                        if arduino_mega.readline()[:-2].decode('utf-8') == "4321" :
                            #arduino_mega.write(str.encode("#4321$"))
                            arduino_connected = True
            if not krpc_connected :
                try :
                    conn = krpc.connect(name='Controller', address='127.0.0.1', rpc_port=50000, stream_port=50001)
                except ConnectionRefusedError : 
                    print('Krpc introuvable')
                    time.sleep(.5)
                else :
                    krpc_connected = True
            if krpc_connected and arduino_connected :
                vessel = conn.space_center.active_vessel
                start = vessel.reference_frame
                ready = True
                disconnect = False

    def __init__(self):
        self.vSF = 0
        self.vLF = 0
        self.vOX = 0
        self.vEL = 0
        self.vMP = 0

        self.gear = conn.add_stream(getattr, vessel.control, 'gear')
        self.brakes = conn.add_stream(getattr, vessel.control, 'brakes')
        self.solar = conn.add_stream(getattr, vessel.control, 'solar_panels')
        self.chutes = conn.add_stream(getattr, vessel.control, 'parachutes')
        self.lights = conn.add_stream(getattr, vessel.control, 'lights')
        
        
        self.apo_alt = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
        self.apo_time = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
        self.peri_alt  = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
        self.peri_time  = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
        srf_frame = vessel.orbit.body.reference_frame
        srf_flight = vessel.flight(srf_frame)
        self.accel = conn.add_stream(getattr, srf_flight, 'g_force')
        self.surfV = conn.add_stream(getattr, srf_flight, 'speed') # Pas la bonne vitesse
        self.overheating  = 12 #Find percentage !
        self.altitude = 0.0 #Find
        self.machNumber = conn.add_stream(getattr, vessel.flight(), 'mach')
        self.surfAltitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
        self.verticalVelocity = conn.add_stream(getattr, vessel.flight(), 'vertical_speed')

    def jauges(self):
        self.ladder = vessel.control.get_action_group(9)
        self.A1 = vessel.control.get_action_group(0) # Stream ???
        self.A2 = vessel.control.get_action_group(1) # Stream ??
        
        self.A3 = vessel.control.get_action_group(2) # Stream ??
        self.A4 = vessel.control.get_action_group(3) # Stream ??
        self.BP = self.gear() << 7 |self.brakes() << 6|self.solar() << 5|self.chutes() << 4|self.lights() << 3|self.ladder << 2|self.A1 << 1|self.A2

        self.BP2 = self.A3 << 1 | self.A4

        actSF = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1).amount("SolidFuel")
        maxSF = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1).max("SolidFuel")
        if actSF != 0 :
            self.vSF = round((actSF / maxSF)*8)+1
        else :
            self.vSF = 0
        
        actLF = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1).amount("LiquidFuel")
        maxLF = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1).max("LiquidFuel")
        if actLF != 0 :
            self.vLF = round((actLF / maxLF)*8)+1
        else :
            self.vLF = 0
        
        actOX = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1).amount("Oxidizer")
        maxOX = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1).max("Oxidizer")
        if actOX != 0 :
            self.vOX = round((actOX / maxOX)*8)+1
        else :
            self.vOX = 0
        
        actEL = vessel.resources.amount("ElectricCharge")
        maxEL = vessel.resources.max("ElectricCharge")
        if actEL != 0 :
            self.vEL = round((actEL / maxEL)*8)+1
        else :
            self.vEL = 0
        
        actMP = vessel.resources.amount("MonoPropellant")
        maxMP = vessel.resources.max("MonoPropellant")
        if actMP != 0 :
            self.vMP = round((actMP / maxMP)*8)+1
        else :
            self.vMP = 0

    # Node Data
    def timetoManeuver(self):
        if len(vessel.control.nodes) :
            node = vessel.control.nodes[0]
            return node.time_to
        else:
            return 0

    def remainingDV(self):
        if len(vessel.control.nodes) :
            node = vessel.control.nodes[0]
            return  node.remaining_delta_v
        else:
            return 0
    
    def gamescene(self):
        mode = conn.krpc.current_game_scene.name
        if mode == "space_center" :
            return 1
        elif mode == "tracking_station" :
            return 2
        elif mode == "editor_vab" :
            return 3
        elif mode == "editor_sph" :
            return 4
        elif mode == "flight" :
            return 5
    # Target Data
    def targetInfo(self):
        if False : #conn.space_center.target_vessel
            if conn.space_center.target_docking_port == None:
                target = conn.space_center.target_vessel.reference_frame
            else:
                target = conn.space_center.target_docking_port.reference_frame

            if conn.space_center.active_vessel.parts.controlling.docking_port == None:
                current = conn.space_center.active_vessel
                current_position = current.position(target)
                velocity = current.velocity(target)
            else:
                current = conn.space_center.active_vessel.parts.controlling.docking_port
                current_position = current.position(target)
                velocity = current.part.velocity(target)
            
            displacement = np.array(current_position)
            distanceToTarget  = la.norm(displacement)
            relativeVelocityToTarget = la.norm(np.array(velocity))
            return [distanceToTarget,relativeVelocityToTarget]
        else:
            return [0,0]
               
    def preparePacket(self, packetType):
        Base = str(str(packetType) + ";" + str(self.BP) + ";" + str(self.BP2) + ";" + str(self.vSF) + ";" + str(self.vLF) + ";" + str(self.vOX) + ";" + str(self.vEL) + ";" + str(self.vMP))
        if packetType == 0: 
            packet = str.encode( Base + ";" + str(round(self.accel(),1)) + ";" + str(round(self.surfV(),1)) + ";$")
        if packetType == 1: 
            packet = str.encode( Base + ";" + str(self.apo_alt()) + ";" + str(self.peri_alt()) + ";" + str(round(self.apo_time())) + ";" + str(round(self.peri_time())) + ";$")
        if packetType == 2: 
            packet = str.encode( Base + ";" + str(round(self.timetoManeuver())) + ";" + str(round(self.remainingDV(),1)) + ";$")
        if packetType == 3:
            packet = str.encode( Base + ";" + str(round(self.targetInfo()[0],1)) + ";" + str(round(self.targetInfo()[1],1)) + ";$")
        if packetType == 4:
           packet = str.encode( Base + ";" + str(round(self.accel(),1)) + ";" + str(self.overheating) + ";$")
        if packetType == 5:
            packet = str.encode( Base + ";" + str(round(self.surfAltitude(),1)) + ";" + str(round(self.verticalVelocity(),1)) + ";$")
        if packetType == 6:
            packet = str.encode( Base + ";" + str(round(self.altitude,1)) + ";" + str(round(self.machNumber(),1)) + ";$")
        return packet


lasttime = 0
# Infinite loop
time.sleep(2)
last = 0
while True :
    if disconnect:
        KSP.Handshake()
        GameData = KSP()

    mode = GameData.gamescene()
    if mode == 1:
        time.sleep(.5)
    if mode == 2:
        time.sleep(.5)
    if mode == 3:
        time.sleep(.5)
    if mode == 4:
        time.sleep(.5)
    if mode == 5:
        # Incoming Commands
        Controller = arduino_mega.readline()[:-2].decode('utf-8')
        if Controller:
            if "LCD" in Controller: LCDmode = int(Controller[3:])
            elif "Th" in Controller: 
                Throttle = int(Controller[3:])
                if Throttle < 5: 
                    vessel.control.throttle = 0
                else :
                    vessel.control.throttle = Throttle/1023
            elif "TX" in Controller:
                TX = int(Controller[3 :])
                if TX < 480 or TX > 512:
                    vessel.control.right = -(TX-512)/512
                else:
                    vessel.control.right = 0
            elif "TY" in Controller: 
                TY = int(Controller[3:])
                if TY < 473 or TY > 512:
                    vessel.control.up = -(TY-512)/512
                else:
                    vessel.control.up = 0
            elif "TZ" in Controller: 
                TZ = int(Controller[3:])
                #if TZ < 40 or TZ > 52:
                    #vessel.control.forward = (TZ-100)/100
            elif "RX" in Controller:
                RX = int(Controller[3:])
                if RX < 490 or RX > 525:
                    vessel.control.yaw = -(RX-512)/512
                else:
                    vessel.control.yaw = 0
            elif "RY" in Controller: 
                RY = int(Controller[3:])
                if RY < 506 or RY > 520:
                    vessel.control.pitch = (RY-512)/512
                else:
                    vessel.control.pitch = 0
            elif "RZ" in Controller: 
                RZ = int(Controller[3:])
                #if RZ < 65 or RZ > 115:
                #   vessel.control.roll = (RZ-100) /100
            elif Controller == "sas": vessel.control.sas = not vessel.control.sas
            elif Controller == "rcs": vessel.control.rcs = not vessel.control.rcs
            elif Controller == "abort": vessel.control.abort = not vessel.control.abort
            elif Controller == "light": vessel.control.lights = not vessel.control.lights
            elif Controller == "ladder": vessel.control.toggle_action_group(9)
            elif Controller == "solar": vessel.control.solar_panels = not vessel.control.solar_panels
            elif Controller == "chutes": vessel.control.parachutes = True
            elif Controller == "gears": vessel.control.gear = not vessel.control.gear
            elif Controller == "brakes": vessel.control.brakes = not vessel.control.brakes
            elif Controller == "act1": vessel.control.toggle_action_group(0)
            elif Controller == "act2": vessel.control.toggle_action_group(1)
            elif Controller == "act3": vessel.control.toggle_action_group(2)
            elif Controller == "act4": vessel.control.toggle_action_group(3)
            #elif Controller == "pTB": 
            #elif Controller == "pRB": 
            elif Controller == "boutST":
                vessel.control.activate_next_stage()
                time.sleep(.200)
            else :
                print(Controller)
        now = time.time_ns()
        if now - last > 100000000 :
            GameData.jauges()
            packet = GameData.preparePacket(LCDmode)
            print(packet)
            arduino_mega.write(packet)
            last = now
