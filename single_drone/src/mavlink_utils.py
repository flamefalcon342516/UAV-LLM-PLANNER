from pymavlink import mavutil
import time

class MAVVehicle:

    def __init__(self, connection):
        print(f"Connecting to {connection}")
        self.master = mavutil.mavlink_connection(
            connection,
            source_system=250
        )
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=30)
        print(
            f"✓ Connected "
            f"SYSID={self.master.target_system} "
            f"COMPID={self.master.target_component}"
        )
        print(
            f"Connected SYSID={self.master.target_system}"
        )
    @property
    def sysid(self):
        return self.master.target_system
    
    def wait_ack(self, command):
        while True:
            msg = self.master.recv_match(
                type="COMMAND_ACK",
                blocking=True,
                timeout=5,
            )
            if msg is None:
                raise RuntimeError("ACK timeout")
            if msg.command == command:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return
                raise RuntimeError(
                    f"Command rejected ({msg.result})"
                )
            
    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
    
    def arm(self):

        self.master.arducopter_arm()

        self.master.motors_armed_wait()

        print("Armed")
      
    def takeoff(self, altitude):

        self.master.mav.command_long_send(

            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,

            0,

            0,
            0,
            0,
            0,

            0,
            0,
            altitude,
        )    
    def goto(self, lat, lon, alt):

        self.master.mav.set_position_target_global_int_send(

            0,

            self.master.target_system,

            self.master.target_component,

            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,

            0b0000111111111000,

            int(lat * 1e7),
            int(lon * 1e7),
            alt,

            0,
            0,
            0,

            0,
            0,
            0,

            0,
            0
        )    
    def rtl(self):
        self.set_mode("RTL")    