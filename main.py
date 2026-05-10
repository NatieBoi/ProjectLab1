# =========================================================
# ROVER MAIN CONTROL FILE
# =========================================================

# =========================================================
# SECTION 1: IMPORTS
# =========================================================
from machine import Pin, PWM, I2C, ADC, Timer
import time
import math
import struct

from vl53l5cx.mp import VL53L5CXMP
from vl53l5cx import DATA_TARGET_STATUS, DATA_DISTANCE_MM, DATA_SIGNAL_PER_SPAD
from vl53l5cx import STATUS_VALID, RESOLUTION_4X4

from hcsr04 import HCSR04
from ssd1306 import SSD1306_I2C

print("=== ROVER STARTING ===")


# =========================================================
# SECTION 2: COLOR SENSOR CLASS (TCS34725)
# =========================================================
class TCS34725:
    ADDRESS     = 0x29
    COMMAND_BIT = 0x80
    ENABLE      = 0x00
    ATIME       = 0x01
    CONTROL     = 0x0F
    ID          = 0x12
    CDATAL      = 0x14
    RDATAL      = 0x16
    GDATAL      = 0x18
    BDATAL      = 0x1A

    def __init__(self, i2c):
        self.i2c = i2c
        chip_id = self._read(self.ID)
        print("[COLOR] Chip ID: {}".format(hex(chip_id)))
        self._write(self.ATIME, 0xEB)
        self._write(self.CONTROL, 0x01)
        self._write(self.ENABLE, 0x01)
        time.sleep_ms(10)
        self._write(self.ENABLE, 0x03)
        time.sleep_ms(50)
        print("[COLOR] Sensor initialised")

    def _write(self, reg, value):
        self.i2c.writeto(self.ADDRESS, bytes([self.COMMAND_BIT | reg, value]))

    def _read(self, reg):
        self.i2c.writeto(self.ADDRESS, bytes([self.COMMAND_BIT | reg]))
        return self.i2c.readfrom(self.ADDRESS, 1)[0]

    def _read16(self, reg):
        self.i2c.writeto(self.ADDRESS, bytes([self.COMMAND_BIT | reg]))
        data = self.i2c.readfrom(self.ADDRESS, 2)
        return data[0] | (data[1] << 8)

    def get_raw_data(self):
        r = self._read16(self.RDATAL)
        g = self._read16(self.GDATAL)
        b = self._read16(self.BDATAL)
        c = self._read16(self.CDATAL)
        return r, g, b, c


# =========================================================
# SECTION 3: MOTOR SETUP
# =========================================================
def percent_to_duty(percentage):
    if percentage < 0:     percentage = 0
    elif percentage > 100: percentage = 100
    return int((65535 * percentage) / 100)

# Motor RIGHT
sr  = Pin(7,  Pin.OUT)
pnr = Pin(8,  Pin.OUT)
enr = PWM(Pin(9))
enr.freq(20000)

# Motor LEFT
sl  = Pin(10, Pin.OUT)
pnl = Pin(11, Pin.OUT)
enl = PWM(Pin(12))
enl.freq(20000)

# Encoders — signed: positive = forward, negative = backward
enc_left  = 0
enc_right = 0

# Direction flags — set by motor functions BEFORE enabling PWM
_dir_left  = 1
_dir_right = 1

def _enc_left_irq(pin):
    global enc_left
    enc_left += _dir_left

def _enc_right_irq(pin):
    global enc_right
    enc_right += _dir_right

def reset_encoders():
    global enc_left, enc_right, _last_enc_left, _last_enc_right
    enc_left        = 0
    enc_right       = 0
    _last_enc_left  = 0
    _last_enc_right = 0

# Constants
MAX_SPD        = 70
SYNC_KP        = 2.0
GYRO_KP        = 0.18
GYRO_DEADBAND  = 3.0
PULSES_PER_REV = 12
WHEEL_CIRC_MM  = 188.5
MM_PER_PULSE   = WHEEL_CIRC_MM / PULSES_PER_REV
TRACK_WIDTH_MM = 153.0
MOTOR_TRIM     = 1.0

# Wake drivers
sr.high()
sl.high()
time.sleep_ms(100)
print("[MOTOR] Drivers awake. MM_PER_PULSE={:.2f}".format(MM_PER_PULSE))


# =========================================================
# SECTION 4: SENSOR I2C BUS
# =========================================================
i2c_sensor = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)
print("[I2C] Sensor bus scan: {}".format([hex(x) for x in i2c_sensor.scan()]))


# =========================================================
# SECTION 5: IMU SETUP
# =========================================================
i2c_sensor.writeto_mem(0x6A, 0x10, bytes([0x40]))
i2c_sensor.writeto_mem(0x6A, 0x11, bytes([0x40]))
time.sleep_ms(100)

print("[IMU] Calibrating gyro — keep still...")
acc = 0
for _ in range(300):
    data = i2c_sensor.readfrom_mem(0x6A, 0x26, 2)
    acc += struct.unpack_from('<h', data)[0]
    time.sleep_ms(5)
GYRO_BIAS = (acc / 300) * 8.75e-3
print("[IMU] Gyro bias={:.4f} deg/s".format(GYRO_BIAS))

gyro_smooth = 0.0
GYRO_ALPHA  = 0.8
cached_gz   = 0.0

def gyro_z():
    global gyro_smooth
    data = i2c_sensor.readfrom_mem(0x6A, 0x26, 2)
    raw  = struct.unpack_from('<h', data)[0] * 8.75e-3 - GYRO_BIAS
    gyro_smooth = GYRO_ALPHA * raw + (1 - GYRO_ALPHA) * gyro_smooth
    return gyro_smooth


# =========================================================
# SECTION 5B: FIELD DIMENSIONS
# =========================================================
FIELD_W_MM    = 3658
FIELD_H_MM    = 2134
ROBOT_SIZE_MM = 315
ROBOT_HALF    = ROBOT_SIZE_MM // 2
GOAL_DEPTH_MM = 305
GOAL_WIDTH_MM = 914
GOAL_Y_CENTER = FIELD_H_MM / 2
GOAL_Y_MIN    = GOAL_Y_CENTER - GOAL_WIDTH_MM / 2
GOAL_Y_MAX    = GOAL_Y_CENTER + GOAL_WIDTH_MM / 2
GOAL_LINE_MARGIN_MM      = GOAL_DEPTH_MM + ROBOT_SIZE_MM
FIELD_BOUNDARY_MARGIN_MM = 250

SEARCH_CENTER_X_MM = FIELD_W_MM / 2
SEARCH_CENTER_Y_MM = FIELD_H_MM / 2

# Center line X — goalie must not cross this
CENTER_LINE_X_MM = FIELD_W_MM / 2

INITIAL_DRIVE_TO_CENTER_MM = int(
    (FIELD_W_MM / 2 - ROBOT_HALF - 150) / math.cos(math.radians(35)))
SECOND_LEG_MM = 1200

ENEMY_GOAL_X_MM = FIELD_W_MM - GOAL_DEPTH_MM
OUR_GOAL_X_MM   = GOAL_DEPTH_MM

# Shot trigger distances — robot must be within these to fire
# FIX: original values of 10mm and 30mm were far too small given odometry drift
NEAR_GOAL_SHOOT_MM     = 350
FAR_GOAL_REPOSITION_MM = 550

print("[FIELD] W={}mm H={}mm center=({:.0f},{:.0f})".format(
    FIELD_W_MM, FIELD_H_MM, SEARCH_CENTER_X_MM, SEARCH_CENTER_Y_MM))
print("[FIELD] Initial drive={}mm  second leg={}mm".format(
    INITIAL_DRIVE_TO_CENTER_MM, SECOND_LEG_MM))


# =========================================================
# SECTION 5C: STARTING POSITIONS + ROLE
# =========================================================
_sw_pos  = Pin(0, Pin.IN, Pin.PULL_UP)
_sw_role = Pin(1, Pin.IN, Pin.PULL_UP)

START_POSITIONS = {
    0: (ROBOT_HALF, ROBOT_HALF,              0.0, "B1-BACK-LEFT"),
    1: (ROBOT_HALF, FIELD_H_MM - ROBOT_HALF, 0.0, "B2-BACK-RIGHT"),
}

_sel = 1 if _sw_pos.value() else 0
_sp  = START_POSITIONS[_sel]
START_POS_ID = "B1" if _sel == 0 else "B2"

# Role: HIGH = attacker/kicker, LOW = goalie/defender
IS_KICKER = bool(_sw_role.value())
ROLE_STR  = "KICKER" if IS_KICKER else "GOALIE"

print("[START] Position={} Role={}".format(_sp[3], ROLE_STR))

pos_x       = float(_sp[0])
pos_y       = float(_sp[1])
heading_deg = 0.0

_last_enc_left  = 0
_last_enc_right = 0

_pos_print_counter = 0
_POS_PRINT_EVERY   = 50
_pivot_mode = False


def update_position(dt):
    global pos_x, pos_y, heading_deg, _last_enc_left, _last_enc_right
    global _pos_print_counter

    dl = (enc_left  - _last_enc_left)  * MM_PER_PULSE
    dr = (enc_right - _last_enc_right) * MM_PER_PULSE
    _last_enc_left  = enc_left
    _last_enc_right = enc_right

    if _pivot_mode:
        return

    d_heading = math.degrees((dr - dl) / TRACK_WIDTH_MM)
    heading_deg += d_heading

    while heading_deg >  180: heading_deg -= 360
    while heading_deg < -180: heading_deg += 360

    dist = (dl + dr) / 2.0
    h = math.radians(heading_deg)
    pos_x += dist * math.cos(h)
    pos_y += dist * math.sin(h)
    pos_x = max(0, min(FIELD_W_MM, pos_x))
    pos_y = max(0, min(FIELD_H_MM, pos_y))

    _pos_print_counter += 1
    if _pos_print_counter >= _POS_PRINT_EVERY:
        _pos_print_counter = 0
        print("[POS] x={:.0f}mm y={:.0f}mm hdg={:.1f}deg encL={} encR={}".format(
            pos_x, pos_y, heading_deg, enc_left, enc_right))


# =========================================================
# SECTION 5D: CACHED VALUES + FLAGS
# =========================================================
cached_r, cached_g, cached_b, cached_c = 0, 0, 0, 0
cached_color_name = "NONE"
sonar_index       = 0
cached_direction  = None
cached_avg_dist   = None

# Phase timing — used by _phase_tick() and _resync_loop_timer()
_phase_last_us = time.ticks_us()


# =========================================================
# SECTION 5E: TIMERS + ENCODER IRQS
# =========================================================
flag_sonar = False
flag_oled  = False
flag_map   = False
flag_color = False

def _timer_sonar_cb(t):
    global flag_sonar
    flag_sonar = True

def _timer_oled_cb(t):
    global flag_oled
    flag_oled = True

def _timer_map_cb(t):
    global flag_map
    flag_map = True

def _timer_color_cb(t):
    global flag_color
    flag_color = True

timer_sonar = Timer()
timer_oled  = Timer()
timer_map   = Timer()
timer_color = Timer()

timer_sonar.init(period=80,   mode=Timer.PERIODIC, callback=_timer_sonar_cb)
timer_oled.init( period=1000, mode=Timer.PERIODIC, callback=_timer_oled_cb)
timer_map.init(  period=100,  mode=Timer.PERIODIC, callback=_timer_map_cb)
timer_color.init(period=30,   mode=Timer.PERIODIC, callback=_timer_color_cb)

ENC_LEFT  = Pin(13, Pin.IN, Pin.PULL_UP)
ENC_RIGHT = Pin(14, Pin.IN, Pin.PULL_UP)
ENC_LEFT.irq(trigger=Pin.IRQ_RISING,  handler=_enc_left_irq)
ENC_RIGHT.irq(trigger=Pin.IRQ_RISING, handler=_enc_right_irq)
print("[ENC] Encoder IRQs attached")


# =========================================================
# SECTION 6: MOTOR FUNCTIONS
# =========================================================
def motorR_forward(duty):
    global _dir_right
    _dir_right = 1
    sr.high(); pnr.high(); enr.duty_u16(duty)

def motorR_backward(duty):
    global _dir_right
    _dir_right = -1
    sr.high(); pnr.low(); enr.duty_u16(duty)

def motorR_stop():
    sr.low(); pnr.low(); enr.duty_u16(0)

def motorL_forward(duty):
    global _dir_left
    _dir_left = 1
    sl.high(); pnl.high(); enl.duty_u16(duty)

def motorL_backward(duty):
    global _dir_left
    _dir_left = -1
    sl.high(); pnl.low(); enl.duty_u16(duty)

def motorL_stop():
    sl.low(); pnl.low(); enl.duty_u16(0)

def brake():
    motorR_stop()
    motorL_stop()

def shutdown():
    brake()

def _phase_tick():
    """Real-dt position update for use inside phase loops."""
    global cached_gz, _phase_last_us
    now_us = time.ticks_us()
    dt = time.ticks_diff(now_us, _phase_last_us) * 1e-6
    _phase_last_us = now_us
    cached_gz = gyro_z()
    update_position(dt)
    update_map()
    return dt

def _resync_loop_timer():
    """Call before returning from any phase to the main loop."""
    global _last_loop_ms, _phase_last_us
    _last_loop_ms  = time.ticks_ms()
    _phase_last_us = time.ticks_us()

def _drive_synced(speed, duration_ms):
    t_start    = time.ticks_ms()
    base       = min(abs(speed), MAX_SPD)
    sign       = 1 if speed > 0 else -1
    local_hdg  = 0.0
    last_us    = time.ticks_us()
    last_enc_l = enc_left
    last_enc_r = enc_right

    while time.ticks_diff(time.ticks_ms(), t_start) < duration_ms:
        t_wait = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t_wait) < 20:
            pass
        now_us  = time.ticks_us()
        dt      = time.ticks_diff(now_us, last_us) * 1e-6
        last_us = now_us
        gz = gyro_z()
        if abs(gz) > 0.5:
            local_hdg += gz * dt
        update_position(dt)
        curr_l = enc_left
        curr_r = enc_right
        enc_error  = abs(curr_l - last_enc_l) - abs(curr_r - last_enc_r)
        last_enc_l = curr_l
        last_enc_r = curr_r
        enc_corr   = int(SYNC_KP * enc_error)
        hdg_err    = local_hdg if abs(local_hdg) >= GYRO_DEADBAND else 0.0
        gyro_corr  = int(GYRO_KP * hdg_err)
        correction = enc_corr + gyro_corr
        l_spd = max(0, min(100, base - correction))
        r_spd = max(0, min(100, base + correction))
        if sign > 0:
            motorL_forward(percent_to_duty(int(l_spd * MOTOR_TRIM)))
            motorR_forward(percent_to_duty(r_spd))
        else:
            motorL_backward(percent_to_duty(int(l_spd * MOTOR_TRIM)))
            motorR_backward(percent_to_duty(r_spd))
        if check_overcurrent():
            print("[OC] Emergency brake in drive")
            brake(); return
    brake()

def _encoder_drive(dist_mm, speed=50):
    """Drive exactly dist_mm using encoder pulse counting."""
    target_pulses = int(dist_mm / MM_PER_PULSE)
    reset_encoders()
    local_hdg  = 0.0
    last_us    = time.ticks_us()
    last_enc_l = enc_left
    last_enc_r = enc_right
    print("[MOT] encoder_drive dist={}mm target={} pulses".format(dist_mm, target_pulses))

    while (enc_left + enc_right) / 2.0 < target_pulses:
        t_wait = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t_wait) < 20:
            pass
        now_us  = time.ticks_us()
        dt      = time.ticks_diff(now_us, last_us) * 1e-6
        last_us = now_us
        cached_gz = gyro_z()
        if abs(cached_gz) > 0.5:
            local_hdg += cached_gz * dt
        update_position(dt)
        update_map()
        curr_l = enc_left
        curr_r = enc_right
        enc_error  = abs(curr_l - last_enc_l) - abs(curr_r - last_enc_r)
        last_enc_l = curr_l
        last_enc_r = curr_r
        enc_corr   = int(SYNC_KP * enc_error)
        hdg_err    = local_hdg if abs(local_hdg) >= GYRO_DEADBAND else 0.0
        gyro_corr  = int(GYRO_KP * hdg_err)
        correction = enc_corr + gyro_corr
        l_spd = max(0, min(100, speed - correction))
        r_spd = max(0, min(100, speed + correction))
        motorL_forward(percent_to_duty(int(l_spd * MOTOR_TRIM)))
        motorR_forward(percent_to_duty(r_spd))
        if check_overcurrent():
            brake(); return
    brake()
    print("[MOT] encoder_drive done pos=({:.0f},{:.0f})".format(pos_x, pos_y))

def correct_heading_to_zero(max_deg=15, speed=35):
    drift = heading_deg
    while drift >  180: drift -= 360
    while drift < -180: drift += 360
    if abs(drift) < 2.0:
        return
    drift = max(-max_deg, min(max_deg, drift))
    turn_ms = int(abs(drift) * 12)
    print("[HDG] Correcting {:.1f}deg ({} ms)".format(drift, turn_ms))
    if drift > 0:
        motorR_forward(percent_to_duty(speed))
        motorL_backward(percent_to_duty(speed))
    else:
        motorL_forward(percent_to_duty(speed))
        motorR_backward(percent_to_duty(speed))
    time.sleep_ms(turn_ms)
    brake()
    time.sleep_ms(30)

def forward(speed=70, duration_ms=500):
    _drive_synced(speed, duration_ms)

def backward(speed=70, duration_ms=500):
    _drive_synced(-speed, duration_ms)

def turn(degrees, speed=40):
    global cached_gz, heading_deg, _pivot_mode, _last_enc_left, _last_enc_right
    last_us = time.ticks_us()
    t_start = time.ticks_ms()
    base    = min(abs(speed), MAX_SPD)
    target  = abs(degrees)
    turned  = 0.0

    _pivot_mode = True

    while turned < target:
        if time.ticks_diff(time.ticks_ms(), t_start) > 10000:
            break
        now_us  = time.ticks_us()
        dt      = time.ticks_diff(now_us, last_us) * 1e-6
        last_us = now_us
        cached_gz = gyro_z()
        delta = abs(cached_gz) * dt
        turned += delta

        if degrees > 0:
            heading_deg += delta
            motorL_forward(percent_to_duty(base))
            motorR_backward(percent_to_duty(base))
        else:
            heading_deg -= delta
            motorR_forward(percent_to_duty(base))
            motorL_backward(percent_to_duty(base))

        while heading_deg >  180: heading_deg -= 360
        while heading_deg < -180: heading_deg += 360

        t_wait = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t_wait) < 20:
            pass
        if check_overcurrent():
            print("[OC] Emergency brake in turn")
            brake(); break

    brake()
    _pivot_mode = False
    _last_enc_left  = enc_left
    _last_enc_right = enc_right
    print("[MOT] turn done — turned={:.1f}deg hdg={:.1f}deg".format(turned, heading_deg))

def turn_to_face(tx, ty, speed=35):
    global cached_gz, heading_deg, _pivot_mode, enc_left, enc_right, _last_enc_left, _last_enc_right
    dx = tx - pos_x
    dy = ty - pos_y
    if abs(dx) < 1.0 and abs(dy) < 1.0:
        return
    target_heading = math.degrees(math.atan2(dy, dx))

    # Wait for gyro to settle before sampling heading
    time.sleep_ms(80)
    for _ in range(5):
        gyro_z()
        time.sleep_ms(10)

    def _heading_error():
        e = target_heading - heading_deg
        while e >  180: e -= 360
        while e < -180: e += 360
        return e
    ...

    error = _heading_error()
    if abs(error) < 20.0:
        return

    print("[TTF] Turning {:.1f}deg to face ({:.0f},{:.0f})".format(error, tx, ty))
    last_us = time.ticks_us()
    t_start = time.ticks_ms()
    _pivot_mode = True
    settled_ms  = None

    while True:
        if time.ticks_diff(time.ticks_ms(), t_start) > 10000:
            print("[TTF] Timeout — snapping heading to target")
            # Snap to where we were trying to go so subsequent calls aren't corrupted
            heading_deg = target_heading
            break

        error = _heading_error()

        if abs(error) < 20.0:
            if settled_ms is None:
                settled_ms = time.ticks_ms()
                brake()
            elif time.ticks_diff(time.ticks_ms(), settled_ms) >= 250:
                # Snap to target on clean exit too — gyro drift during slow turns accumulates
                heading_deg = target_heading
                break
            now_us  = time.ticks_us()
            dt      = time.ticks_diff(now_us, last_us) * 1e-6
            last_us = now_us
            cached_gz    = gyro_z()
            heading_deg += cached_gz * dt
            while heading_deg >  180: heading_deg -= 360
            while heading_deg < -180: heading_deg += 360
            time.sleep_ms(15)
            continue

        settled_ms = None
        spd = max(20, min(speed, int(abs(error) * 0.8)))
        if abs(error) < 30:
            spd = 20
        now_us  = time.ticks_us()
        dt      = time.ticks_diff(now_us, last_us) * 1e-6
        last_us = now_us
        cached_gz    = gyro_z()
        heading_deg += cached_gz * dt
        while heading_deg >  180: heading_deg -= 360
        while heading_deg < -180: heading_deg += 360
        update_map()
        if error > 0:
            motorL_backward(percent_to_duty(spd))
            motorR_forward(percent_to_duty(spd))
        else:
            motorL_forward(percent_to_duty(spd))
            motorR_backward(percent_to_duty(spd))
        time.sleep_ms(15)
        if check_overcurrent():
            brake(); break

    brake()
    _pivot_mode = False
    enc_left        = 0
    enc_right       = 0
    _last_enc_left  = 0
    _last_enc_right = 0
    print("[TTF] Done — hdg={:.1f}deg".format(heading_deg))


def turn_left(degrees=90):  turn(-degrees)
def turn_right(degrees=90): turn(degrees)
def reverse(speed=70, duration_ms=500): backward(speed, duration_ms)


# =========================================================
# SECTION 7: SERVO + SOLENOID
# =========================================================
solenoid = Pin(20, Pin.OUT)
servo    = PWM(Pin(15))
servo.freq(50)

SERVO_OPEN   = 20
SERVO_CLOSED = 120

def set_angle(angle):
    pulse_us = 500 + (2500 - 500) * angle / 180
    servo.duty_u16(int(pulse_us * 65535 / 20000))

def open_gate():
    set_angle(SERVO_OPEN)
    print("[SERVO] Gate OPEN")

def close_gate():
    set_angle(SERVO_CLOSED)
    print("[SERVO] Gate CLOSED")

gate_open = True
kicking   = "OPEN"

def fire_solenoid():
    # FIX: trimmed pre/post delays from 2000ms each to 300ms — saves ~4s per shot
    print("[KICK] Nudging forward before fire")
    motorL_forward(percent_to_duty(30))
    motorR_forward(percent_to_duty(30))
    time.sleep_ms(150)
    brake()
    time.sleep_ms(300)
    print("[KICK] Opening gate")
    open_gate()
    time.sleep_ms(500)
    print("[KICK] Firing solenoid!")
    solenoid.high()
    time.sleep_ms(500)
    solenoid.low()
    print("[KICK] Solenoid off")


# =========================================================
# SECTION 8: LIDAR SETUP
# =========================================================
lpn = Pin(6, Pin.OUT, value=1)
time.sleep_ms(50)

i2c_lidar = I2C(0, scl=Pin(5), sda=Pin(4), freq=1000000)
print("[LIDAR] I2C scan: {}".format([hex(x) for x in i2c_lidar.scan()]))

tof = VL53L5CXMP(i2c_lidar, lpn=lpn)
tof.reset()
tof.init()
tof.resolution  = RESOLUTION_4X4
tof.ranging_freq = 5
tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS, DATA_SIGNAL_PER_SPAD})
print("[LIDAR] Ranging started")


# =========================================================
# SECTION 9: COLOR SENSOR + OLED SETUP
# =========================================================
color_sensor = TCS34725(i2c_sensor)

width  = 128
height = 64
oledL = SSD1306_I2C(width, height, i2c_sensor, addr=0x3D)
oledR = SSD1306_I2C(width, height, i2c_sensor, addr=0x3C)
oledL.fill(0); oledL.text("LEFT",  0, 0); oledL.show()
oledR.fill(0); oledR.text("RIGHT", 0, 0); oledR.show()
print("[OLED] Both displays initialised")


# =========================================================
# SECTION 10: ULTRASONIC SENSORS
# =========================================================
sonic_front = HCSR04(trig_pin=16, echo_pin=17, min_dist=20, max_dist=1000, temp_c=22.0)
sonic_left  = HCSR04(trig_pin=16, echo_pin=22, min_dist=20, max_dist=1000, temp_c=22.0)
sonic_right = HCSR04(trig_pin=16, echo_pin=28, min_dist=20, max_dist=1000, temp_c=22.0)

SONAR_OBSTACLE_MM      = 200
SONAR_BOUNDARY_MM      = 100
SONAR_FLOOR_MIN_MM     = 50
GUARD_ROVER_DETECT_MM  = 300

sonar_dist_front = None
sonar_dist_left  = None
sonar_dist_right = None

def read_all_sonars():
    global sonar_dist_front, sonar_dist_left, sonar_dist_right
    try:
        r = sonic_front.read()
        sonar_dist_front = int(r.distance) if r.ok() else None
    except: sonar_dist_front = None
    try:
        r = sonic_left.read()
        sonar_dist_left = int(r.distance) if r.ok() else None
    except: sonar_dist_left = None
    try:
        r = sonic_right.read()
        sonar_dist_right = int(r.distance) if r.ok() else None
    except: sonar_dist_right = None

def obstacle_too_close():
    if sonar_dist_front is not None and SONAR_FLOOR_MIN_MM < sonar_dist_front < SONAR_OBSTACLE_MM: return True
    if sonar_dist_left  is not None and SONAR_FLOOR_MIN_MM < sonar_dist_left  < SONAR_OBSTACLE_MM: return True
    if sonar_dist_right is not None and SONAR_FLOOR_MIN_MM < sonar_dist_right < SONAR_OBSTACLE_MM: return True
    return False

def near_goal_line():
    return pos_x < GOAL_LINE_MARGIN_MM or pos_x > (FIELD_W_MM - GOAL_LINE_MARGIN_MM)


# =========================================================
# SECTION 11: TOF / BALL DETECTION SETTINGS
# =========================================================
SMOOTH_FRAMES    = 5
BALL_D_MIN       = 120
BALL_D_MAX       = 500
BALL_MIN_SIGNAL  = 0
BALL_CONTRAST_MM = 40
BALL_CLUSTER_MM  = 80

_smooth_buf      = [[] for _ in range(16)]
_smooth_sig_buf  = [[] for _ in range(16)]
_smooth_valid    = [False] * 16
_smoothed_signal = [0] * 16


# =========================================================
# SECTION 12: CURRENT SENSING
# =========================================================
adc_left  = ADC(Pin(26))
adc_right = ADC(Pin(27))

IPROPI_RATIO            = 2000
R_ADC_IMPEDANCE         = 60000
ADC_VREF                = 3.3
ADC_FULL_SCALE          = 65535
OVERCURRENT_LIMIT_A     = 2.0
OVERCURRENT_COOLDOWN_MS = 2000
_last_overcurrent_ms    = 0

def _adc_to_amps(raw):
    return (raw / ADC_FULL_SCALE * ADC_VREF / R_ADC_IMPEDANCE) * IPROPI_RATIO

def check_overcurrent():
    global _last_overcurrent_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_overcurrent_ms) < OVERCURRENT_COOLDOWN_MS:
        return False
    al = _adc_to_amps(adc_left.read_u16())
    ar = _adc_to_amps(adc_right.read_u16())
    tripped = False
    if al > OVERCURRENT_LIMIT_A:
        print("[OC] LEFT {:.2f}A".format(al)); motorL_stop(); tripped = True
    if ar > OVERCURRENT_LIMIT_A:
        print("[OC] RIGHT {:.2f}A".format(ar)); motorR_stop(); tripped = True
    if tripped:
        _last_overcurrent_ms = now
    return tripped


# =========================================================
# SECTION 13: TOF FLOOR FILTER + BALL DETECTION
# =========================================================
def update_smooth(distance, status, signal):
    smoothed = [0] * 16
    for i in range(16):
        if status[i] == STATUS_VALID:
            _smooth_buf[i].append(distance[i])
            if len(_smooth_buf[i]) > SMOOTH_FRAMES: _smooth_buf[i].pop(0)
            if signal is not None:
                _smooth_sig_buf[i].append(signal[i])
                if len(_smooth_sig_buf[i]) > SMOOTH_FRAMES: _smooth_sig_buf[i].pop(0)
        if _smooth_buf[i]:
            smoothed[i]      = sum(_smooth_buf[i]) // len(_smooth_buf[i])
            _smooth_valid[i] = True
        else:
            smoothed[i] = 0; _smooth_valid[i] = False
        _smoothed_signal[i] = (sum(_smooth_sig_buf[i]) // len(_smooth_sig_buf[i])
                               if _smooth_sig_buf[i] else 0)
    return smoothed

def _median(values):
    if not values: return 0
    s = sorted(values); n = len(s); mid = n // 2
    return s[mid] if n % 2 == 1 else (s[mid - 1] + s[mid]) // 2

def _pixel_passes_floor_filter(col, smoothed):
    idx = 8 + col
    if not _smooth_valid[idx]: return False
    d = smoothed[idx]
    if not (BALL_D_MIN <= d <= BALL_D_MAX): return False
    row2 = [smoothed[8 + c] for c in range(4) if _smooth_valid[8 + c]]
    if len(row2) >= 2 and (_median(row2) - d) < BALL_CONTRAST_MM: return False
    return True

def find_ball(smoothed):
    if smoothed is None: return None, None
    active = {}
    for col in range(4):
        if _pixel_passes_floor_filter(col, smoothed):
            active[col] = smoothed[8 + col]
    if not active: return None, None
    closest_dist = active[min(active, key=lambda c: active[c])]
    active = {c: d for c, d in active.items() if d <= closest_dist + BALL_CLUSTER_MM}
    cols     = set(active.keys())
    avg_dist = sum(active.values()) // len(active)
    if cols == {0,3} or cols == {0,3,1} or cols == {0,3,2} or cols == {0,1,2,3}:
        return "CENTER", avg_dist
    if cols == {1, 2}:
        return ("LEFT", active[1]) if active[1] <= active[2] else ("RIGHT", active[2])
    if cols <= {0, 1}: return "LEFT",  avg_dist
    if cols <= {2, 3}: return "RIGHT", avg_dist
    wc = sum(col * active[col] for col in active) / sum(active.values())
    return ("LEFT", avg_dist) if wc < 1.5 else ("RIGHT", avg_dist)

_tof_print_counter = 0
_TOF_PRINT_EVERY   = 10

def poll_lidar():
    global cached_direction, cached_avg_dist, smoothed, _tof_print_counter
    if tof.check_data_ready():
        try:
            results  = tof.get_ranging_data()
            distance = results.distance_mm
            status   = results.target_status
            signal   = getattr(results, 'signal_per_spad', None)
            smoothed = update_smooth(distance, status, signal)
            prev_dir = cached_direction
            cached_direction, cached_avg_dist = find_ball(smoothed)
            _tof_print_counter += 1
            if _tof_print_counter >= _TOF_PRINT_EVERY:
                _tof_print_counter = 0
                print("[TOF] dir={} dist={}mm".format(cached_direction, cached_avg_dist))
            if prev_dir is None and cached_direction is not None:
                print("[TOF] *** BALL ACQUIRED: {} @ {}mm ***".format(
                    cached_direction, cached_avg_dist))
            elif prev_dir is not None and cached_direction is None:
                print("[TOF] *** BALL LOST ***")
        except Exception as e:
            print("[TOF] ERROR: {}".format(e))


# =========================================================
# SECTION 14: COLOR DETECTION
# =========================================================
_black_r=180;  _black_g=280;  _black_b=420;  _black_c=850
_white_r=700;  _white_g=2000; _white_b=1600; _white_c=4200

def _normalize(value, low, high):
    if high <= low: return 0
    v = (value - low) * 255 // (high - low)
    return max(0, min(255, v))

def detect_color(r, g, b, c):
    rn = _normalize(r, _black_r, _white_r)
    gn = _normalize(g, _black_g, _white_g)
    bn = _normalize(b, _black_b, _white_b)
    total = rn + gn + bn if (rn + gn + bn) > 0 else 1
    rp = rn * 100 // total
    gp = gn * 100 // total
    bp = bn * 100 // total

    if c < 1100:
        result = "NONE"
    elif rn < 60:
        result = "NONE"
    elif rp > 45 and rn > gn and rn > bn:
        result = "RED"
    elif gp > 45 and gn > rn and gn > bn:
        result = "GREEN"
    elif bp > 35 and bn > rn:
        result = "BLUE"
    else:
        result = "NOT RED"
    return result, rn, gn, bn, rp, gp, bp

_color_print_counter = 0
_COLOR_PRINT_EVERY   = 25

def poll_color():
    global cached_r, cached_g, cached_b, cached_c, cached_color_name
    global _color_print_counter
    try:
        cached_r, cached_g, cached_b, cached_c = color_sensor.get_raw_data()
        result, rn, gn, bn, rp, gp, bp = detect_color(cached_r, cached_g, cached_b, cached_c)
        cached_color_name = result
        _color_print_counter += 1
        if _color_print_counter >= _COLOR_PRINT_EVERY:
            _color_print_counter = 0
            print("[COLOR] R={} G={} B={} C={} result={}".format(
                cached_r, cached_g, cached_b, cached_c, cached_color_name))
    except Exception as e:
        print("[COLOR] ERROR: {}".format(e))
        cached_color_name = "NONE"


# =========================================================
# SECTION 14B: POSSESSION CONFIRMATION
# =========================================================
_color_confirm_count  = 0
_color_confirm_needed = 2
_color_confirmed_last = "NONE"
_color_last_seen_ms   = 0
COLOR_HOLD_MS         = 400

def get_confirmed_ball_color():
    global _color_confirm_count, _color_confirmed_last, _color_last_seen_ms
    current = cached_color_name
    now     = time.ticks_ms()

    if current == "NONE":
        if _color_confirmed_last != "NONE":
            if time.ticks_diff(now, _color_last_seen_ms) < COLOR_HOLD_MS:
                return _color_confirmed_last
        _color_confirm_count  = 0
        _color_confirmed_last = "NONE"
        return None

    _color_last_seen_ms = now

    if current == _color_confirmed_last:
        _color_confirm_count += 1
    else:
        _color_confirmed_last = current
        _color_confirm_count  = 1

    if _color_confirm_count >= _color_confirm_needed:
        print("[COLOR] *** CONFIRMED: {} ***".format(current))
        return current
    return None

def reset_color_confirmation():
    global _color_confirm_count, _color_confirmed_last, _color_last_seen_ms
    _color_confirm_count  = 0
    _color_confirmed_last = "NONE"
    _color_last_seen_ms   = 0


# =========================================================
# SECTION 15: OBSTACLE AVOIDANCE
# =========================================================
def avoid_obstacle():
    global cached_gz, _prev_rx, _prev_ry
    front = sonar_dist_front is not None and sonar_dist_front < SONAR_OBSTACLE_MM
    left  = sonar_dist_left  is not None and sonar_dist_left  < SONAR_OBSTACLE_MM
    right = sonar_dist_right is not None and sonar_dist_right < SONAR_OBSTACLE_MM
    print("[OBS] F={} L={} R={} pos=({:.0f},{:.0f})".format(
        sonar_dist_front, sonar_dist_left, sonar_dist_right, pos_x, pos_y))

    def _timed(ms):
        global cached_gz, _prev_rx, _prev_ry
        t0 = time.ticks_ms(); lu = time.ticks_us()
        while time.ticks_diff(time.ticks_ms(), t0) < ms:
            nu = time.ticks_us(); dt = time.ticks_diff(nu, lu) * 1e-6; lu = nu
            cached_gz = gyro_z(); update_position(dt); update_map()
            time.sleep_ms(10)
        brake(); time.sleep_ms(50)

    motorL_backward(percent_to_duty(50))
    motorR_backward(percent_to_duty(50))
    _timed(400)

    if front and left and right:
        if pos_x < FIELD_W_MM / 2:
            motorL_forward(percent_to_duty(40)); motorR_backward(percent_to_duty(40))
        else:
            motorR_forward(percent_to_duty(40)); motorL_backward(percent_to_duty(40))
        _timed(900)
    elif front and left and not right:
        motorL_forward(percent_to_duty(40)); motorR_backward(percent_to_duty(40)); _timed(500)
    elif front and right and not left:
        motorR_forward(percent_to_duty(40)); motorL_backward(percent_to_duty(40)); _timed(500)
    elif front:
        if pos_x < FIELD_W_MM / 2:
            motorL_forward(percent_to_duty(40)); motorR_backward(percent_to_duty(40))
        else:
            motorR_forward(percent_to_duty(40)); motorL_backward(percent_to_duty(40))
        _timed(500)
    elif left:
        motorL_forward(percent_to_duty(40)); motorR_backward(percent_to_duty(40)); _timed(400)
    elif right:
        motorR_forward(percent_to_duty(40)); motorL_backward(percent_to_duty(40)); _timed(400)

    brake()
    print("[OBS] Done pos=({:.0f},{:.0f}) hdg={:.1f}".format(pos_x, pos_y, heading_deg))


# =========================================================
# SECTION 16: OLED MAP
# =========================================================
PAD_X, PAD_Y = 4, 4
MAP_W, MAP_H = 120, 56

def mm_to_px(x, y):
    return (int(PAD_X + (x / FIELD_W_MM) * MAP_W),
            int(PAD_Y + MAP_H - (y / FIELD_H_MM) * MAP_H))

GOAL_DEPTH_PX = max(2, int((GOAL_DEPTH_MM / FIELD_W_MM) * MAP_W))
GOAL_OPEN_PX  = max(4, int((GOAL_WIDTH_MM / FIELD_H_MM) * MAP_H))
GOAL_L_X      = PAD_X
GOAL_L_Y      = PAD_Y + (MAP_H - GOAL_OPEN_PX) // 2
GOAL_R_X      = PAD_X + MAP_W - GOAL_DEPTH_PX
GOAL_R_Y      = GOAL_L_Y
_prev_rx = PAD_X + 2
_prev_ry = PAD_Y + 2

def draw_static_map():
    oledR.fill(0)
    oledR.rect(PAD_X, PAD_Y, MAP_W, MAP_H, 1)
    oledR.vline(PAD_X + MAP_W // 2, PAD_Y, MAP_H, 1)
    oledR.rect(GOAL_L_X, GOAL_L_Y, GOAL_DEPTH_PX, GOAL_OPEN_PX, 1)
    oledR.rect(GOAL_R_X, GOAL_R_Y, GOAL_DEPTH_PX, GOAL_OPEN_PX, 1)
    oledR.text(START_POS_ID, 0, 0)
    oledR.show()

def _draw_robot_x(rx, ry, color):
    oledR.pixel(rx - 1, ry - 1, color)
    oledR.pixel(rx + 1, ry - 1, color)
    oledR.pixel(rx,     ry,     color)
    oledR.pixel(rx - 1, ry + 1, color)
    oledR.pixel(rx + 1, ry + 1, color)

def update_map():
    global _prev_rx, _prev_ry
    rx, ry = mm_to_px(pos_x, pos_y)
    rx = max(PAD_X + 2, min(PAD_X + MAP_W - 3, rx))
    ry = max(PAD_Y + 2, min(PAD_Y + MAP_H - 3, ry))
    if (rx, ry) != (_prev_rx, _prev_ry):
        _draw_robot_x(_prev_rx, _prev_ry, 0)
        _draw_robot_x(rx, ry, 1)
        oledR.show()
        _prev_rx, _prev_ry = rx, ry


# =========================================================
# SECTION 17: STARTUP STATE
# =========================================================
open_gate()
solenoid.low()

draw_static_map()
_init_rx, _init_ry = mm_to_px(pos_x, pos_y)
_init_rx = max(PAD_X + 2, min(PAD_X + MAP_W - 3, _init_rx))
_init_ry = max(PAD_Y + 2, min(PAD_Y + MAP_H - 3, _init_ry))
_draw_robot_x(_init_rx, _init_ry, 1)
oledR.show()
_prev_rx, _prev_ry = _init_rx, _init_ry

oledL.fill(0)
oledL.text(ROLE_STR, 0, 0)
oledL.text(START_POS_ID, 0, 12)
oledL.show()

smoothed = None
print("[INIT] Startup complete. Robot at ({:.0f},{:.0f})".format(pos_x, pos_y))


# =========================================================
# SECTION 18: BOUNDARY GUARD
# =========================================================
boundary_interrupt_enabled = False
_last_boundary_check_ms    = 0
BOUNDARY_CHECK_PERIOD_MS   = 1000

def field_boundary_guard():
    if not boundary_interrupt_enabled: return
    if near_goal_line(): return

    # FIX: goalie must not cross the center line — enforce hard limit here
    if not IS_KICKER and pos_x >= CENTER_LINE_X_MM - ROBOT_HALF:
        print("[WALL] Goalie at center line — backing off")
        brake()
        backward(speed=40, duration_ms=400)
        _resync_loop_timer()
        return

    too_close = False; wall = ""
    if pos_x < FIELD_BOUNDARY_MARGIN_MM:
        wall = "LEFT x={:.0f}".format(pos_x); too_close = True
    elif pos_x > FIELD_W_MM - FIELD_BOUNDARY_MARGIN_MM:
        wall = "RIGHT x={:.0f}".format(pos_x); too_close = True
    if pos_y < FIELD_BOUNDARY_MARGIN_MM:
        wall += " BOTTOM y={:.0f}".format(pos_y); too_close = True
    elif pos_y > FIELD_H_MM - FIELD_BOUNDARY_MARGIN_MM:
        wall += " TOP y={:.0f}".format(pos_y); too_close = True
    if not too_close: return
    print("[WALL] Too close: {} — steering to center".format(wall))
    brake()

    # Goalie steers to own-side center, kicker steers to field center
    if IS_KICKER:
        turn_to_face(SEARCH_CENTER_X_MM, SEARCH_CENTER_Y_MM)
    else:
        turn_to_face(OUR_GOAL_X_MM + FIELD_W_MM // 4, SEARCH_CENTER_Y_MM)


# =========================================================
# SECTION 19: SPIN HELPER
# =========================================================
def _do_full_spin():
    global cached_gz, heading_deg, _pivot_mode, _last_enc_left, _last_enc_right
    print("[SPIN] Starting full 360deg spin")
    heading_before = heading_deg
    spin_heading   = 0.0
    turned         = 0.0
    last_us        = time.ticks_us()

    _pivot_mode = True

    while turned < 360:
        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, last_us) * 1e-6; last_us = now_us
        cached_gz     = gyro_z()
        spin_heading += cached_gz * dt
        turned       += abs(cached_gz) * dt
        update_map()
        field_boundary_guard()
        poll_lidar()

        if cached_direction is not None:
            _pivot_mode = False
            heading_deg = heading_before + spin_heading
            while heading_deg >  180: heading_deg -= 360
            while heading_deg < -180: heading_deg += 360
            brake()
            _last_enc_left  = enc_left
            _last_enc_right = enc_right
            print("[SPIN] Ball found — dir={} dist={}mm hdg={:.1f}deg".format(
                cached_direction, cached_avg_dist, heading_deg))
            return True

        poll_color()
        confirmed = get_confirmed_ball_color()
        if confirmed is not None:
            _pivot_mode = False
            heading_deg = heading_before + spin_heading
            while heading_deg >  180: heading_deg -= 360
            while heading_deg < -180: heading_deg += 360
            brake()
            _last_enc_left  = enc_left
            _last_enc_right = enc_right
            close_gate()
            print("[SPIN] Ball in gate during spin — color={}".format(confirmed))
            return True

        if START_POS_ID == "B1":
            motorL_forward(percent_to_duty(30)); motorR_backward(percent_to_duty(30))
        else:
            motorR_forward(percent_to_duty(30)); motorL_backward(percent_to_duty(30))
        time.sleep_ms(20)

    brake()
    _pivot_mode = False
    _last_enc_left  = enc_left
    _last_enc_right = enc_right
    heading_deg = heading_before + spin_heading
    while heading_deg >  180: heading_deg -= 360
    while heading_deg < -180: heading_deg += 360
    print("[SPIN] 360 complete — no ball. hdg={:.1f}deg".format(heading_deg))
    return False


# =========================================================
# SECTION 20: INITIAL SWEEP  (kicker only)
# =========================================================
_search_spinning    = True
_search_turned_deg  = 0.0

def phase_initial_sweep():
    global boundary_interrupt_enabled, _last_boundary_check_ms
    global _search_spinning, _search_turned_deg
    print("=== PHASE: INITIAL SWEEP [{}] ===".format(START_POS_ID))
    boundary_interrupt_enabled = False

    if START_POS_ID == "B1":
        turn_right(35)
    else:
        turn_left(35)
    time.sleep_ms(150)

    print("[SWEEP] Driving {}mm to pre-center".format(INITIAL_DRIVE_TO_CENTER_MM))
    _encoder_drive(INITIAL_DRIVE_TO_CENTER_MM, speed=50)
    poll_lidar()

    if cached_direction is not None:
        print("[SWEEP] Ball spotted on leg 1")
        boundary_interrupt_enabled = True
        _last_boundary_check_ms = time.ticks_ms()
        _search_spinning = True; _search_turned_deg = 0.0
        _resync_loop_timer(); return

    time.sleep_ms(150)
    if _do_full_spin():
        boundary_interrupt_enabled = True
        _last_boundary_check_ms = time.ticks_ms()
        _search_spinning = True; _search_turned_deg = 0.0
        _resync_loop_timer(); return

    correct_heading_to_zero(); time.sleep_ms(150)
    _encoder_drive(SECOND_LEG_MM, speed=50)
    poll_lidar()

    if cached_direction is not None:
        print("[SWEEP] Ball spotted on leg 2")
        boundary_interrupt_enabled = True
        _last_boundary_check_ms = time.ticks_ms()
        _search_spinning = True; _search_turned_deg = 0.0
        _resync_loop_timer(); return

    time.sleep_ms(150)
    if _do_full_spin():
        boundary_interrupt_enabled = True
        _last_boundary_check_ms = time.ticks_ms()
        _search_spinning = True; _search_turned_deg = 0.0
        _resync_loop_timer(); return

    print("[SWEEP] Complete — no ball found. Handing to main loop.")
    boundary_interrupt_enabled = True
    _last_boundary_check_ms = time.ticks_ms()
    _search_spinning = False; _search_turned_deg = 0.0
    _resync_loop_timer()


# =========================================================
# SECTION 20B: GOAL SELECTION
# =========================================================
def select_target_goal(ball_color):
    """Return (target_x, target_y, label) for where to bring this ball.

    Scoring: red = -2, green = +2, blue = +1.
    All balls deposited in the ENEMY goal benefit your team:
      - Red at enemy goal removes a -2 threat from your side
      - Green/blue at enemy goal adds points directly
    There is never a good reason for the kicker to bring a ball to its own goal.
    Goalie mode handles its own goal separately (eject, never deposit).
    """
    return ENEMY_GOAL_X_MM, GOAL_Y_CENTER, "ENEMY"

def _dist_to_goal(gx, gy):
    dx = gx - pos_x
    dy = gy - pos_y
    return math.sqrt(dx * dx + dy * dy)


# =========================================================
# SECTION 20C: GOALIE STARTUP
# =========================================================
# Goalie patrol Y positions — pace between these two points
GOALIE_PATROL_Y_LOW  = GOAL_Y_MIN + 150
GOALIE_PATROL_Y_HIGH = GOAL_Y_MAX - 150
# Goalie holds this X — just in front of the goal mouth
GOALIE_GUARD_X       = OUR_GOAL_X_MM + ROBOT_SIZE_MM + 50
# Goalie max X — never go past this toward center
GOALIE_MAX_X         = CENTER_LINE_X_MM - ROBOT_HALF - 50

_goalie_patrol_dir = 1   # +1 = moving toward HIGH, -1 = moving toward LOW


def phase_goalie_startup():
    """Drive goalie to its guard position in front of own goal."""
    global boundary_interrupt_enabled, _last_boundary_check_ms
    print("=== PHASE: GOALIE STARTUP ===")
    boundary_interrupt_enabled = False

    # Face the guard position and drive there
    turn_to_face(GOALIE_GUARD_X, SEARCH_CENTER_Y_MM)
    time.sleep_ms(100)
    dist = _dist_to_goal(GOALIE_GUARD_X, SEARCH_CENTER_Y_MM)
    if dist > 100:
        _encoder_drive(int(dist), speed=50)

    # Face toward field center (looking down-field)
    turn_to_face(CENTER_LINE_X_MM, SEARCH_CENTER_Y_MM)
    time.sleep_ms(100)

    boundary_interrupt_enabled = True
    _last_boundary_check_ms = time.ticks_ms()
    _resync_loop_timer()
    print("[GOALIE] Startup complete at ({:.0f},{:.0f})".format(pos_x, pos_y))


# =========================================================
# SECTION 25: OLED BALL DISPLAY
# =========================================================
def update_oled_ball(target_x=None, target_y=None, target_label=""):
    oledL.fill(0)

    # Row 0: role + colour reading
    oledL.text(ROLE_STR[:3], 0, 0)
    oledL.text(cached_color_name[:5], 72, 0)

    # Row 1: ball direction or NO BALL
    if cached_direction is None:
        oledL.text("NO BALL", 20, 12)
    else:
        label = {"CENTER": ("CTR", 44), "LEFT": ("LEFT", 40), "RIGHT": ("RGT", 44)}
        txt, x = label.get(cached_direction, ("?", 56))
        oledL.text(txt, x, 12)
        if cached_avg_dist is not None:
            oledL.text("{}mm".format(cached_avg_dist), 0, 12)

    # Row 2: current position X Y
    oledL.text("X{:.0f} Y{:.0f}".format(pos_x, pos_y)[:16], 0, 26)

    # Row 3: heading
    oledL.text("HDG:{:.0f}".format(heading_deg)[:16], 0, 38)

    # Row 4: distance to current target goal
    if target_x is not None:
        gdist = _dist_to_goal(target_x, target_y)
        oledL.text("{}G {:.0f}mm".format(target_label[0], gdist)[:16], 0, 50)
    else:
        de = _dist_to_goal(ENEMY_GOAL_X_MM, GOAL_Y_CENTER)
        oledL.text("EG:{:.0f}".format(de)[:16], 0, 50)

    oledL.show()


# =========================================================
# SECTION 21: AGRO MODE  (kicker — drive ball to enemy goal)
# =========================================================
def _enemy_rover_blocking():
    return sonar_dist_front is not None and sonar_dist_front < GUARD_ROVER_DETECT_MM

def phase_agro_mode(target_x, target_y, target_label):
    global _phase_last_us, _pivot_mode
    print("=== PHASE: AGRO MODE — target {} ({:.0f},{:.0f}) ===".format(
        target_label, target_x, target_y))
    AGRO_TIMEOUT_MS = 30000
    t_start = time.ticks_ms()
    _pivot_mode    = False
    _phase_last_us = time.ticks_us()

    TARGET_X = target_x
    TARGET_Y = target_y

    while time.ticks_diff(time.ticks_ms(), t_start) < AGRO_TIMEOUT_MS:
        _phase_tick()
        read_all_sonars()

        if obstacle_too_close():
            avoid_obstacle()
            _phase_last_us = time.ticks_us()
            continue

        poll_color()
        poll_lidar()
        update_oled_ball(TARGET_X, TARGET_Y, target_label)

        if cached_color_name == "NONE":
            print("[AGRO] Lost possession — aborting")
            open_gate()
            _resync_loop_timer()
            return

        dx = TARGET_X - pos_x
        dy = TARGET_Y - pos_y
        dist_to_goal = math.sqrt(dx * dx + dy * dy)

        y_error = abs(pos_y - TARGET_Y)
        y_threshold = max(150, dist_to_goal * 0.15)
        if y_error > y_threshold and dist_to_goal < 800:
            print("[AGRO] Y drift={:.0f}mm — re-aiming".format(y_error))
            brake()
            time.sleep_ms(100)
            turn_to_face(TARGET_X, TARGET_Y)
            _phase_last_us = time.ticks_us()
            time.sleep_ms(100)
    # Drive a short burst toward goal before re-evaluating
    # so we don't immediately re-trigger the same y_error check
            forward(speed=45, duration_ms=400)
            _phase_last_us = time.ticks_us()
            continue
        
        if _enemy_rover_blocking():
            if dist_to_goal <= NEAR_GOAL_SHOOT_MM:
                fire_solenoid()
                _resync_loop_timer()
                return
            wait_start = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), wait_start) < 7000:
                read_all_sonars(); poll_lidar(); update_map()
                time.sleep_ms(200)
                if not _enemy_rover_blocking(): break
            if _enemy_rover_blocking():
                read_all_sonars()
                if (sonar_dist_left or 0) >= (sonar_dist_right or 0):
                    turn_left(25); forward(40, 500); turn_right(25)
                else:
                    turn_right(25); forward(40, 500); turn_left(25)
                _phase_last_us = time.ticks_us()
        else:
            if dist_to_goal <= NEAR_GOAL_SHOOT_MM:
                fire_solenoid()
                _resync_loop_timer()
                return
            elif dist_to_goal <= FAR_GOAL_REPOSITION_MM:
                brake()
                fire_solenoid()
                _resync_loop_timer()
                return
            else:
                forward(speed=55, duration_ms=300)
                _phase_last_us = time.ticks_us()

        time.sleep_ms(20)

    brake()
    print("[AGRO] Timeout — firing fallback")
    fire_solenoid()
    _resync_loop_timer()


# =========================================================
# SECTION 22: GOALIE MODE  (NEW — replaces carry-to-own-goal)
# =========================================================
def phase_goalie_loop():
    """
    Goalie main loop. Runs forever (called from main loop instead of kicker logic).

    Behaviour priority:
      1. If a ball is confirmed in the gate → eject it away from own goal
      2. If LiDAR sees a ball coming → intercept (move to match ball's Y, then push away)
      3. Otherwise → patrol between GOALIE_PATROL_Y_LOW and GOALIE_PATROL_Y_HIGH
         at the guard X, always facing field center
      4. Hard constraint: never cross CENTER_LINE_X_MM
    """
    global _phase_last_us, _pivot_mode, _goalie_patrol_dir

    _phase_last_us = time.ticks_us()
    _pivot_mode    = False

    GOALIE_LOOP_TIMEOUT_MS = 200   # ms per iteration — keeps loop responsive

    print("=== GOALIE LOOP ITERATION ===")

    _phase_tick()
    read_all_sonars()

    if obstacle_too_close():
        avoid_obstacle()
        _resync_loop_timer()
        return

    poll_color()
    poll_lidar()
    update_oled_ball()

    # ── 1. POSSESSION: eject ball away from own goal ────────────────────
    ball_in_gate = get_confirmed_ball_color()
    if ball_in_gate is not None:
        print("[GOALIE] Ball in gate ({}) — ejecting toward center".format(ball_in_gate))
        close_gate()
        time.sleep_ms(200)
        # Turn to face field center and push ball away
        turn_to_face(CENTER_LINE_X_MM, SEARCH_CENTER_Y_MM)
        time.sleep_ms(100)
        # Drive forward a short distance to clear the goal mouth
        _encoder_drive(400, speed=50)
        time.sleep_ms(100)
        # Fire solenoid to eject
        open_gate()
        time.sleep_ms(200)
        solenoid.high()
        time.sleep_ms(500)
        solenoid.low()
        print("[GOALIE] Ball ejected")
        reset_color_confirmation()
        # Retreat back to guard position
        turn_to_face(GOALIE_GUARD_X, pos_y)
        _encoder_drive(min(400, int(_dist_to_goal(GOALIE_GUARD_X, pos_y))), speed=45)
        turn_to_face(CENTER_LINE_X_MM, SEARCH_CENTER_Y_MM)
        _resync_loop_timer()
        return

    # ── 2. INTERCEPT: ball visible on LiDAR ─────────────────────────────
    if cached_direction is not None and cached_avg_dist is not None:
        print("[GOALIE] Intercepting ball dir={} dist={}mm".format(
            cached_direction, cached_avg_dist))

        # Estimate ball Y from LiDAR direction
        if cached_direction == "LEFT":
            intercept_y = pos_y - 150
        elif cached_direction == "RIGHT":
            intercept_y = pos_y + 150
        else:
            intercept_y = pos_y

        intercept_y = max(GOALIE_PATROL_Y_LOW, min(GOALIE_PATROL_Y_HIGH, intercept_y))

        # Clamp X to guard position — don't advance toward the ball
        intercept_x = GOALIE_GUARD_X

        # Only move laterally if the offset is meaningful
        y_err = intercept_y - pos_y
        if abs(y_err) > 80:
            # Lateral slide: short timed drive while staying at guard X
            spd = 40
            duration = min(600, int(abs(y_err) * 0.8))
            if y_err > 0:
                # Move toward higher Y — depends on which side we drive
                motorL_forward(percent_to_duty(spd))
                motorR_forward(percent_to_duty(spd))
            else:
                motorL_backward(percent_to_duty(spd))
                motorR_backward(percent_to_duty(spd))
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < duration:
                _phase_tick()
                # Hard goalie boundary — bail if drifting past guard X
                if pos_x > GOALIE_MAX_X:
                    break
                time.sleep_ms(15)
            brake()
            time.sleep_ms(50)

        # Re-face field center after lateral move
        turn_to_face(CENTER_LINE_X_MM, SEARCH_CENTER_Y_MM)
        _resync_loop_timer()
        return

    # ── 3. PATROL: no ball visible ───────────────────────────────────────
    patrol_target_y = (GOALIE_PATROL_Y_HIGH if _goalie_patrol_dir > 0
                       else GOALIE_PATROL_Y_LOW)
    y_err = patrol_target_y - pos_y

    if abs(y_err) < 100:
        # Reached this patrol end — flip direction
        _goalie_patrol_dir *= -1
        patrol_target_y = (GOALIE_PATROL_Y_HIGH if _goalie_patrol_dir > 0
                           else GOALIE_PATROL_Y_LOW)
        y_err = patrol_target_y - pos_y

    # Nudge toward patrol target
    spd = 30
    duration = min(400, int(abs(y_err) * 0.5))
    if y_err > 0:
        motorL_forward(percent_to_duty(spd))
        motorR_forward(percent_to_duty(spd))
    else:
        motorL_backward(percent_to_duty(spd))
        motorR_backward(percent_to_duty(spd))

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < duration:
        _phase_tick()
        if pos_x > GOALIE_MAX_X:
            break
        time.sleep_ms(15)
    brake()

    # Always re-face field center after patrol nudge
    turn_to_face(CENTER_LINE_X_MM, SEARCH_CENTER_Y_MM)
    _resync_loop_timer()


# =========================================================
# SECTION 23: POST-SHOT SCAN
# =========================================================
def phase_post_shot_scan():
    global _search_spinning, _search_turned_deg
    print("=== PHASE: POST-SHOT SCAN ===")
    brake()
    time.sleep_ms(300)
    reset_color_confirmation()
    _search_spinning = True
    _search_turned_deg = 0.0
    turn_left(30);  poll_lidar()
    if cached_direction is not None:
        _resync_loop_timer(); return
    turn_right(60); poll_lidar()
    if cached_direction is not None:
        _resync_loop_timer(); return
    turn_left(30)
    _resync_loop_timer()


# =========================================================
# SECTION 24: SEARCH
# =========================================================
def _drive_toward_center():
    dx = SEARCH_CENTER_X_MM - pos_x
    dy = SEARCH_CENTER_Y_MM - pos_y
    dist_to_center = math.sqrt(dx * dx + dy * dy)
    target_angle = math.degrees(math.atan2(dy, dx))
    turn_needed = target_angle - heading_deg
    while turn_needed >  180: turn_needed -= 360
    while turn_needed < -180: turn_needed += 360
    print("[SEARCH] Drive to center dist={:.0f}mm turn={:.1f}deg".format(
        dist_to_center, turn_needed))
    turn(turn_needed, speed=40); time.sleep_ms(150)
    field_boundary_guard()

    target_pulses = int(800 / MM_PER_PULSE)
    reset_encoders()
    local_hdg = 0.0; last_us = time.ticks_us()
    last_enc_l = enc_left; last_enc_r = enc_right

    while (enc_left + enc_right) / 2.0 < target_pulses:
        t_wait = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t_wait) < 20: pass
        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, last_us) * 1e-6; last_us = now_us
        cached_gz = gyro_z(); local_hdg += cached_gz * dt
        update_position(dt); update_map()
        field_boundary_guard(); poll_lidar()
        if cached_direction is not None:
            brake()
            print("[SEARCH] Ball found driving to center")
            return True
        curr_l = enc_left; curr_r = enc_right
        enc_error = abs(curr_l - last_enc_l) - abs(curr_r - last_enc_r)
        last_enc_l = curr_l; last_enc_r = curr_r
        enc_corr  = int(SYNC_KP * enc_error)
        hdg_err   = local_hdg if abs(local_hdg) >= GYRO_DEADBAND else 0.0
        gyro_corr = int(GYRO_KP * hdg_err)
        correction = enc_corr + gyro_corr
        l_spd = max(0, min(100, 50 - correction))
        r_spd = max(0, min(100, 50 + correction))
        motorL_forward(percent_to_duty(int(l_spd * MOTOR_TRIM)))
        motorR_forward(percent_to_duty(r_spd))
        if check_overcurrent(): brake(); break
    brake()
    return False

def search_for_ball_rotate():
    global _search_spinning, _search_turned_deg
    if cached_direction is not None:
        return
    if _search_spinning:
        found = _do_full_spin()
        if found: _search_spinning = True; _search_turned_deg = 0.0; return
        _search_spinning = False
    else:
        found = _drive_toward_center()
        if found: _search_spinning = True; _search_turned_deg = 0.0; return
        _search_spinning = True


# =========================================================
# SECTION 26: MAIN LOOP
# =========================================================
_resync_loop_timer()
_last_loop_ms = time.ticks_ms()

DRIVE_TIMEOUT_MS = 8000

print("=== STARTING MAIN LOOP ===")

try:
    if IS_KICKER:
        # ── KICKER STARTUP: sweep field looking for first ball ───────────
        phase_initial_sweep()
        print("=== INITIAL SWEEP COMPLETE — ENTERING KICKER MAIN LOOP ===")
    else:
        # ── GOALIE STARTUP: drive to guard position ──────────────────────
        phase_goalie_startup()
        print("=== GOALIE STARTUP COMPLETE — ENTERING GOALIE MAIN LOOP ===")

    _resync_loop_timer()
    _last_loop_ms = time.ticks_ms()

    while True:
        now_ms = time.ticks_ms()
        dt     = time.ticks_diff(now_ms, _last_loop_ms) / 1000.0
        if dt > 0.5:
            dt = 0.5
        _last_loop_ms = now_ms

        # ── PRIORITY 1: POSITION + MAP ──────────────────────────────────
        try:
            cached_gz = gyro_z()
            update_position(dt)
        except Exception as e:
            print("[ERR] Gyro: {}".format(e))

        update_map()

        # ── PRIORITY 2: SONAR + OBSTACLE ────────────────────────────────
        if flag_sonar:
            flag_sonar = False
            try:
                if sonar_index == 0:
                    r = sonic_front.read()
                    sonar_dist_front = int(r.distance) if r.ok() else None
                elif sonar_index == 1:
                    r = sonic_left.read()
                    sonar_dist_left  = int(r.distance) if r.ok() else None
                else:
                    r = sonic_right.read()
                    sonar_dist_right = int(r.distance) if r.ok() else None
            except: pass
            sonar_index = (sonar_index + 1) % 3

        if obstacle_too_close():
            brake()
            read_all_sonars()
            avoid_obstacle()
            _resync_loop_timer()
            _last_loop_ms = time.ticks_ms()
            continue

        field_boundary_guard()

        # ── PRIORITY 3: COLOR ────────────────────────────────────────────
        if flag_color:
            flag_color = False
            poll_color()

        # ── PRIORITY 4: TOF ──────────────────────────────────────────────
        poll_lidar()

        # ── OLED ─────────────────────────────────────────────────────────
        if flag_oled:
            flag_oled = False
            update_oled_ball()

        if flag_map:
            flag_map = False
            update_map()

        # ── DECISION TREE ────────────────────────────────────────────────
        if not IS_KICKER:
            # ════════════════════════════════════════════════════════════
            # GOALIE BRANCH — never tries to chase or score
            # ════════════════════════════════════════════════════════════
            phase_goalie_loop()

        else:
            # ════════════════════════════════════════════════════════════
            # KICKER BRANCH
            # ════════════════════════════════════════════════════════════
            if cached_direction is None:
                search_for_ball_rotate()

            else:
                drive_dist_mm  = (cached_avg_dist if cached_avg_dist is not None else 300) + 200
                target_pulses  = int(drive_dist_mm / MM_PER_PULSE)
                print("[MAIN] Ball {} @ {}mm — driving {}mm".format(
                    cached_direction, cached_avg_dist, drive_dist_mm))

                reset_color_confirmation()
                reset_encoders()
                local_hdg  = 0.0
                last_us    = time.ticks_us()
                last_enc_l = enc_left
                last_enc_r = enc_right
                ball_color = None
                base       = 18
                drive_t_start = time.ticks_ms()

                if cached_direction == "LEFT":
                    turn_left(10)
                elif cached_direction == "RIGHT":
                    turn_right(10)

                reset_encoders()
                last_enc_l = enc_left
                last_enc_r = enc_right

                while (enc_left + enc_right) / 2.0 < target_pulses:
                    if time.ticks_diff(time.ticks_ms(), drive_t_start) > DRIVE_TIMEOUT_MS:
                        print("[DRIVE] Timeout — aborting chase")
                        brake()
                        break

                    t_wait = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), t_wait) < 20:
                        pass
                    now_us = time.ticks_us()
                    dt2    = time.ticks_diff(now_us, last_us) * 1e-6
                    last_us = now_us

                    cached_gz = gyro_z(); local_hdg += cached_gz * dt2
                    update_position(dt2); update_map()

                    if obstacle_too_close():
                        brake(); read_all_sonars(); avoid_obstacle(); break

                    field_boundary_guard()

                    poll_color()
                    poll_color()
                    ball_color = get_confirmed_ball_color()
                    if ball_color is not None:
                        brake()
                        close_gate()
                        print("[DRIVE] Color confirmed: {} — gate closed".format(ball_color))
                        time.sleep_ms(300)
                        break

                    poll_lidar()

                    curr_l = enc_left; curr_r = enc_right
                    enc_error  = abs(curr_l - last_enc_l) - abs(curr_r - last_enc_r)
                    last_enc_l = curr_l; last_enc_r = curr_r
                    enc_corr   = int(SYNC_KP * enc_error)
                    hdg_err    = local_hdg if abs(local_hdg) >= GYRO_DEADBAND else 0.0
                    gyro_corr  = int(GYRO_KP * hdg_err)
                    correction = enc_corr + gyro_corr
                    l_spd = max(0, min(100, base - correction))
                    r_spd = max(0, min(100, base + correction))
                    motorL_forward(percent_to_duty(int(l_spd * MOTOR_TRIM)))
                    motorR_forward(percent_to_duty(r_spd))
                    if check_overcurrent(): brake(); break
                else:
                    brake()

                _resync_loop_timer()
                _last_loop_ms = time.ticks_ms()

                print("[DRIVE] Done. ball_color={} pos=({:.0f},{:.0f})".format(
                    ball_color, pos_x, pos_y))

                if ball_color is None:
                    print("[DRIVE] Ball lost — backing up to free trap")
                    motorL_backward(percent_to_duty(30))
                    motorR_backward(percent_to_duty(30))
                    time.sleep_ms(600)
                    brake()
                    time.sleep_ms(200)
                    reset_color_confirmation()
                    _search_spinning = True; _search_turned_deg = 0.0
                    open_gate()
                    _resync_loop_timer()
                    _last_loop_ms = time.ticks_ms()
                    continue

                # FIX: all ball colors go to enemy goal — see select_target_goal()
                gx, gy, glabel = select_target_goal(ball_color)
                print("[DECISION] ball={} role={} → target {} goal ({:.0f},{:.0f})".format(
                    ball_color, ROLE_STR, glabel, gx, gy))

                phase_agro_mode(gx, gy, glabel)
                phase_post_shot_scan()
                open_gate()
                _resync_loop_timer()
                _last_loop_ms = time.ticks_ms()

        time.sleep_ms(20)

except KeyboardInterrupt:
    print("=== KEYBOARD INTERRUPT ===")
    timer_sonar.deinit()
    timer_oled.deinit()
    timer_map.deinit()
    timer_color.deinit()

finally:
    shutdown()
    print("=== SHUTDOWN COMPLETE ===")
