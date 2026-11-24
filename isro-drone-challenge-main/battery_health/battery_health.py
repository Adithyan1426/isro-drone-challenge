from pymavlink import mavutil
import time

# Connect to PX4 (companion port)
master = mavutil.mavlink_connection('udp:0.0.0.0:14540')

print("Waiting for connection...")
master.wait_heartbeat()
print("✅ Connected to PX4")

# -------- PARAMETERS -------- #
LOW_BATTERY = 25      # RTL
CRITICAL_BATTERY = 15 # LAND

rtl_triggered = False
land_triggered = False

print("🔄 Monitoring battery...\n")

while True:
    msg = master.recv_match(type='SYS_STATUS', blocking=True)

    if msg:
        battery = msg.battery_remaining
        print(f"🔋 Battery: {battery}%")

        # LOW BATTERY → RTL
        if battery < LOW_BATTERY and not rtl_triggered:
            print("⚠ Low Battery → RETURN TO BASE")

            master.set_mode_apm("RTL")
            rtl_triggered = True

        # CRITICAL → LAND
        if battery < CRITICAL_BATTERY and not land_triggered:
            print("🚨 Critical Battery → LAND")

            master.set_mode_apm("LAND")
            land_triggered = True
            break

    time.sleep(1)
