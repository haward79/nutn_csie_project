
from typing import Union
from math import sin, cos, radians
from time import sleep
from datetime import datetime
from numpy import uint8
import dronekit

class AttitudeType():

    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    ROTATE_LEFT = 5
    ROTATE_RIGHT = 6

    def __init__(self):

        pass


class Copter():

    def __init__(self):

        self.__isLogToFile = True
        self.__isLogToStdout = True
        self.__vehicle = None

        self.connect()


    def connect(self) -> dronekit.Vehicle:

        self.log('Initializing connection ......')

        while self.__vehicle == None:
            try:
                self.__vehicle = dronekit.connect('/dev/ttyAMA0', wait_ready=True, baud=115200)
            except dronekit.APIException:
                self.log('Please check the wiring or keep waiting ......')

        self.log('Getting version info ......')
        self.__vehicle.wait_ready('autopilot_version')
        self.log('Copter connected.')


    def get_firmwareVersion(self) -> str:
        
        if self.__vehicle != None:
            return self.__vehicle.version
        
        return ''


    def get_param(self, paramName: str) -> str:

        value = ''

        if self.__vehicle != None:
            try:
                value = self.__vehicle.parameters[paramName]
            except KeyError:
                pass

        else:
            self.__log_notConnected()

        if value == None:
            value = ''
        else:
            value = str(value)

        return value


    def get_systemId(self) -> uint8:

        id = ''

        if self.__vehicle != None:
            id = self.get_param('SYSID_THISMAV')
        else:
            self.__log_notConnected()

        if len(id) == 0:
            return -1
        else:
            return int(float(id))


    def get_status(self, statusType: str) -> Union[str, bool, dict]:

        if self.__vehicle != None:
            homePos = self.get_home_location()
            home_lat = homePos.lat
            home_lon = homePos.lon
            home_alt = homePos.alt

            pos = self.get_relative_location()
            pos_lat = pos.lat
            pos_lon = pos.lon
            pos_alt = pos.alt

            if statusType == 'isArmable':
                self.log('> WARN: get_status("isArmable") always return TRUE.')
                return True
                #return self.__vehicle.is_armable
            elif statusType == 'flightMode':
                return self.__vehicle.mode.name
            elif statusType == 'isArmed':
                return self.__vehicle.armed
            elif statusType == 'homePos':
                return {'lat': home_lat, 'lon': home_lon, 'alt': home_alt}
            elif statusType == 'relativePos':
                return {'lat': pos_lat, 'lon': pos_lon, 'alt': pos_alt}
            elif statusType == 'battery':
                return self.__vehicle.battery.voltage

        return ''


    def get_relativeAlt(self) -> float:

        alt = -1

        if self.__vehicle != None:
            alt = self.__vehicle.location.global_relative_frame.alt

        return alt


    def get_home_location(self) -> dronekit.LocationGlobal:

        if self.__vehicle != None:
            cmds = self.__vehicle.commands
            cmds.download()

            try:
                cmds.wait_ready()
            except:
                pass
            else:
                if self.__vehicle.home_location != None:
                    return self.__vehicle.home_location
                else:
                    return dronekit.LocationGlobal(0, 0, 0)

        return dronekit.LocationGlobal(0, 0, 0)


    def get_relative_location(self) -> dronekit.LocationGlobalRelative:

        if self.__vehicle != None:
            cmds = self.__vehicle.commands
            cmds.download()

            try:
                cmds.wait_ready()
            except:
                pass
            else:
                if self.__vehicle.location.global_relative_frame != None:
                    self.__vehicle.location.global_relative_frame
                else:
                    return dronekit.LocationGlobalRelative(0, 0, 0)

        return dronekit.LocationGlobalRelative(0, 0, 0)

    
    def changeFlightMode(self, flightMode: str) -> None:

        self.log('Changing flight mode to ' + flightMode + '......')

        if self.__vehicle != None:
            self.__vehicle.mode = dronekit.VehicleMode(flightMode)
            
            self.log('Waiting for flight mode change ......')
            while self.__vehicle.mode != flightMode:
                sleep(0.5)

            self.log('Flight mode changed to ' + flightMode + '.')

        else:
            self.__log_notConnected()


    def arm(self) -> None:

        if self.__vehicle != None:
            self.changeFlightMode('STABILIZE')

            self.log('Checking isArmable flag ......')
            while not self.get_status('isArmable'):
                sleep(0.5)

            self.log('Arming ......')
            self.__vehicle.arm(wait=True)

            self.log('Waiting for arm completion  ......')
            while not self.__vehicle.armed:
                sleep(0.5)

            self.log('Armed successfully.')

        else:
            self.__log_notConnected()


    def disarm(self) -> None:

        if self.__vehicle != None:
            self.log('Disarming ......')
            self.__vehicle.disarm(wait=True)

            self.log('Waiting for disarm completion  ......')
            while self.__vehicle.armed:
                sleep(0.5)

            self.log('Disarmed successfully.')

        else:
            self.__log_notConnected()


    def takeOff(self, altitude: float) -> bool:

        if altitude < 0:
            self.log('Failed to takeoff. Altitude should NOT be a negative value.')
            return False
        
        else:
            if self.__vehicle != None:
                self.log('Run takeoff procedure ......')

                if not self.__vehicle.armed:
                    self.arm()

                oldFlightMode = self.__vehicle.mode.name

                self.changeFlightMode('GUIDED_NOGPS')

                self.log('Taking off to ' + str(altitude) + ' meters ......')

                currentAlt = self.get_relativeAlt()
                throttle = 0.5

                while True:
                    self.log('-------------------------------------------------')
                    prevAlt = currentAlt
                    currentAlt = self.get_relativeAlt()
                    self.log('Current altitude is %.2f m' % (currentAlt))

                    altDiff = currentAlt - prevAlt
                    altRemain = altitude - currentAlt

                    # Reach target alt.
                    if abs(altRemain) < 0.05:
                        break

                    # Keep going up or down.
                    else:
                        throttleCali = 0

                        # Move in small speed.
                        if abs(altDiff) < 0.1:
                            throttleCali = altRemain * 0.01 * (1 - altDiff * 10)

                        # Move in incorrect direction.
                        elif altDiff * altRemain < 0:
                            throttleCali = 0.5 - throttle

                        # Stable.
                        else:
                            throttleCali = 0
                        
                        throttle += throttleCali

                        if throttle > 1:
                            throttle = 1

                        if throttle < 0:
                            throttle = 0

                        self.log('Throttle: ' + str('%.2f' % throttle))

                        self.set_attitude(AttitudeType.STOP, 0.5, throttle)

                self.log('Took off to ' + str(altitude) + ' meters.')

                self.changeFlightMode(oldFlightMode)

                return True
            
            else:
                self.__log_notConnected()
                return False


    def land(self) -> None:

        if self.__vehicle != None:
            self.log('Run landing procedure ......')

            if self.__vehicle.armed:
                self.changeFlightMode('GUIDED_NOGPS')

                while True:
                    currentAlt = self.get_relativeAlt()
                    self.log('Current altitude is %.2f m' % (currentAlt))

                    # Landed.
                    if abs(currentAlt) < 0.05:
                        break

                    # Keep going down.
                    else:
                        self.set_attitude(AttitudeType.STOP, 0.5, 0.4)

                #self.set_attitude(AttitudeType.BACKWARD, 0.1, 0.1)
                self.set_attitude(AttitudeType.STOP, 0.1, 0)
                self.disarm()
                self.log('Copter landed.')

                self.changeFlightMode('STABILIZE')

            else:
                self.log('Landed due to copter not armed.')
            
        else:
            self.__log_notConnected()


    def land_useMode(self) -> None:

        self.changeFlightMode('LAND')
        self.log('Landing ......')

        currentAlt = self.get_relativeAlt()

        while abs(currentAlt) >= 0.05 and self.get_status('isArmed'):
            self.log('Current altitude is %.2f m' % (currentAlt))
            sleep(0.5)

            currentAlt = self.get_relativeAlt()

        self.log('Landed.')


    def reboot(self) -> None:

        if self.__vehicle != None:
            self.log('Rebooting flight control board ......')
            self.__vehicle.reboot()
            self.__vehicle.close()
            self.__vehicle = None

            self.log('Waiting for vehicle boot up ......')
            sleep(10)
            self.connect()

        else:
            self.__log_notConnected()


    def log(self, msg: str):

        if self.__isLogToStdout:
            print(msg)

        if self.__isLogToFile:
            with open('Copter.log', 'a') as fout:
                fout.write('[' + datetime.now().strftime("%Y.%m.%d %H:%M:%S") + '] ' + msg + '\n')

    
    def log_status(self):

        homePos = self.get_status('homePos')
        relativePos = self.get_status('relativePos')

        msg = self.get_status('flightMode') + ' / ' + ('%.2f' % self.get_status('battery')) + ' V\n'
        msg += str(self.get_status('isArmable')) + ' | ' + str(self.get_status('isArmed')) + '\n'
        msg += 'Home: lat / lon / alt : ' + ('%.2f / %.2f / %.2f' % (homePos['lat'], homePos['lon'], homePos['alt'])) + '\n'
        msg += 'Relative: lat / lon / alt : ' + ('%.2f / %.2f / %.2f' % (relativePos['lat'], relativePos['lon'], relativePos['alt'])) + '\n'

        self.log(msg)


    def __log_notConnected(self):

        self.log('> ERROR: Copter NOT connected.')


    def degree_to_quaternion(self, roll: float, pitch: float, yaw: float) -> list:

        t0 = cos(radians(yaw * 0.5))
        t1 = sin(radians(yaw * 0.5))
        t2 = cos(radians(roll * 0.5))
        t3 = sin(radians(roll * 0.5))
        t4 = cos(radians(pitch * 0.5))
        t5 = sin(radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]


    def __set_attitude(self, roll_angel: float, pitch_angel: float, yaw_angel: float, throttle_perc: float) -> None:

        if throttle_perc >= 0 and throttle_perc <= 1:
            if self.__vehicle != None:
                message = self.__vehicle.message_factory.set_attitude_target_encode(
                    0,  # time_boot_ms
                    0,  # target_system
                    0,  # target_component
                    0b00000100,  # type_mask
                    self.degree_to_quaternion(roll_angel, pitch_angel, yaw_angel),  # q
                    0,  # body_roll_rate
                    0,  # body_pitch_rate
                    0,  # body_yaw_rate
                    throttle_perc  # thrust
                )

                self.__vehicle.send_mavlink(message)

            else:
                self.__log_notConnected()
        
        else:
            self.log('Parameter value, throttle_perc, should be in the range [0, 1] !')


    def __set_attitude_loop(self, roll_angel: float, pitch_angel: float, yaw_angel: float, throttle_perc: float, duration_sec: float) -> None:

        ending = datetime.now().timestamp() + duration_sec

        while datetime.now().timestamp() < ending:
            self.__set_attitude(roll_angel, pitch_angel, yaw_angel, throttle_perc)
            sleep(0.1)

        self.__set_attitude(0, 0, 0, throttle_perc)


    def set_attitude(self, attitude: AttitudeType, duration: float, throttle: float = None) -> None:

        THROTTLE = 0.5
        ANGLE_DIFF = 5

        if throttle == None or (throttle < 0 or throttle > 1):
            throttle = THROTTLE

        if duration >= 0:
            if self.__vehicle != None:
                if self.__vehicle.armed:
                    if attitude == AttitudeType.FORWARD:
                        self.__set_attitude_loop(0, -ANGLE_DIFF, 0, throttle, duration)

                    elif attitude == AttitudeType.BACKWARD:
                        self.__set_attitude_loop(0, ANGLE_DIFF, 0, throttle, duration)
                    
                    elif attitude == AttitudeType.LEFT:
                        self.__set_attitude_loop(-ANGLE_DIFF, 0, 0, throttle, duration)

                    elif attitude == AttitudeType.RIGHT:
                        self.__set_attitude_loop(ANGLE_DIFF, 0, 0, throttle, duration)

                    elif attitude == AttitudeType.ROTATE_LEFT:
                        self.__set_attitude_loop(0, 0, ANGLE_DIFF, throttle, duration)

                    elif attitude == AttitudeType.ROTATE_RIGHT:
                        self.__set_attitude_loop(0, 0, ANGLE_DIFF, throttle, duration)

                    else:
                        self.__set_attitude_loop(0, 0, 0, throttle, duration)

                else:
                    self.log('Please arm the copter before execute this function!')

            else:
                self.__log_notConnected()

        else:
            self.log('Parameter value, duration, duration must equal or greater than 0 !')


    def debug_getVehicle(self):

        return self.__vehicle

