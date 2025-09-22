#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PS5 (DualSense) -> PiCar-X — Manual / Line / Parkour + Cliff/Obstacle + Motor-Bias + Extra-Servo (PWM3)
Neu:
- ○ (Circle): Zusatz-Servo auf PWM3 toggeln (OPEN/CLOSE), 1/4 Umdrehung (~90°)
- Hindernis-Erkennung: Burst + Emergency-Stop (schnell)
- Cliff: adaptive Baseline + schneller Debounce
- MANUAL: Rückwärts-Halten (L2) wird nicht vom Ultraschall-Backup unterbrochen
- R2 halten = vorwärts (100%), L2 halten = rückwärts (100%)
- R1 = Audio-Stop, L1 = Schwarz/Weiß umschalten (LINE & PARKOUR)
- Debug nur bei Events
"""

import os, time, threading, subprocess
from os import geteuid
from pyPS4Controller.controller import Controller
from picarx import Picarx
from vilib import Vilib

# ========= Konfiguration =========
# Geschwindigkeiten
MANUAL_SPEED          = 100   # R2/L2 (0..100)
AUTO_SPEED            = 100   # LINE/PARKOUR
CLIFF_BACKUP_SPEED    = 100   # Rückwärts-Speed bei Cliff

# Lenkung
STEER_MAX_DEG         = 30
DEADZONE_JS           = 10
STEER_TRIM_DEG        = 0
SERVO_LEFT_POSITIVE   = True   # True: +Winkel = links

# Motor-Bias (vorwärts + rückwärts je Seite separat)
MOTOR_L_FWD           = 1.00
MOTOR_R_FWD           = 0.85
MOTOR_L_REV           = 1.00
MOTOR_R_REV           = 0.90

# Kamera
CAM_PAN_MAX           = 35
CAM_TILT_MAX          = 35
CAM_PAN_INVERT        = +1
CAM_TILT_INVERT       = +1

# Safety / Sensoren (Ultraschall)
OBSTACLE_STOP_DISTANCE    = 10.0   # cm -> in LINE/PARKOUR stehen & warten (Median)
OBSTACLE_EMERGENCY_DISTANCE = 7.0  # cm -> sofortiger Stopp (Min-Wert)
OBSTACLE_RELEASE_HYST     = 3.0    # cm über STOP-Distanz zum Wiederanfahren
OBSTACLE_SAMPLES          = 4      # Messungen pro Burst
OBSTACLE_SAMPLE_DELAY     = 0.004  # s zwischen Burst-Samples

# Manuell
MANUAL_BACKUP_TIME     = 0.35
CLIFF_BACKUP_TIME      = 0.35
INPUT_LOCK_TIME        = 1.0

# Cliff-Filter (schnell + stabil)
CLIFF_DEBOUNCE_COUNT   = 3
CLIFF_CLEAR_COUNT      = 4
CLIFF_COOLDOWN_TIME    = 0.6
CLIFF_ALPHA_BASE       = 0.05     # Baseline langsam mitführen
CLIFF_RATIO_THR        = 0.40     # < 40% der Baseline = Cliff

# Parkour / Linie
DEADEND_LOST_TIME      = 0.30
UTURN_STEER            = STEER_MAX_DEG
UTURN_TIME             = 1.2
LINE_STEER_OFFSET      = 22
LINE_REF               = None
LINE_CLIFF_REF         = [200,200,200]
PARKOUR_RIGHT_HAND     = True

# Loop-Timing
MAIN_LOOP_SLEEP        = 0.02     # 20 ms -> schnelle Reaktion

# ---- Zusatz-Servo (PWM3) ----
EXTRA_SERVO_CHANNEL    = 3        # dein PWM3
EXTRA_SERVO_OPEN_ANGLE = +90      # OPEN = +90° (≈ 1/4 Umdrehung)
EXTRA_SERVO_CLOSE_ANGLE= 0        # CLOSE = 0°
EXTRA_SERVO_INVERT     = False    # True falls Richtung vertauscht ist

# Stream
CAM_VFLIP = False
CAM_HFLIP = False
CAM_LOCAL = True
CAM_WEB   = True

# Audio
SOUND_X_PATH      = "/home/yeti/picar-x/example/test/feuer_x.mp3"
SOUND_SQUARE_PATH = "/home/yeti/picar-x/example/test/verlassen.m4a"
# SOUND_CIRCLE_PATH = ...  # Kreis ist jetzt Servo!
BAYERN1_URL       = "https://dispatcher.rndfnk.com/br/br1/nbopf/mp3/mid"
START_VOLUME      = 200
JS_DEVICE         = "/dev/input/js0"


# ========= Audio-Player =========
class Player:
    def __init__(self, start_volume=START_VOLUME):
        self.proc=None; self.has_api=False
        self._inst=None; self._mp=None
        try:
            import vlc
            self._inst = vlc.Instance("--no-video","--aout=alsa","--quiet")
            self._mp = self._inst.media_player_new()
            self._mp.audio_set_volume(int(start_volume))
            self.has_api=True
        except Exception:
            pass
    def _spawn(self, media):
        root=(geteuid()==0)
        cmd=(["vlc-wrapper","-I","dummy","--no-video","--aout=alsa",media] if root
             else ["cvlc","--no-video","--intf","dummy","--aout=alsa",media])
        self.proc=subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    def play_file(self,p):
        self.stop()
        if self.has_api:
            try: m=self._inst.media_new_path(p); self._mp.set_media(m); self._mp.play(); return
            except Exception: self.has_api=False
        if os.path.isfile(p): self._spawn(p)
        else: print("[Audio] file not found:", p)
    def play_url(self,u):
        self.stop()
        if self.has_api:
            try: m=self._inst.media_new(u); self._mp.set_media(m); self._mp.play(); return
            except Exception: self.has_api=False
        self._spawn(u)
    def stop(self):
        if self.has_api:
            try: self._mp.stop()
            except Exception: pass
        if self.proc and self.proc.poll() is None:
            try: self.proc.terminate()
            except Exception: pass
        self.proc=None


# ========= Utils =========
def scale_axis(val, dead=DEADZONE_JS, max_in=32767):
    if abs(val) < dead: return 0.0
    s = (abs(val) - dead) / (max_in - dead)
    s = max(0.0, min(1.0, s))
    return s if val > 0 else -s

def safe_ultra_read(px):
    try:
        d = float(px.ultrasonic.read())
        return d if d > 0 else None
    except Exception:
        return None

def fast_ultra_burst(px, n=OBSTACLE_SAMPLES, delay=OBSTACLE_SAMPLE_DELAY):
    """Mehrfach lesen -> (min, median). Reagiert schnell und trotzdem robust."""
    ds = []
    for _ in range(max(1, n)):
        d = safe_ultra_read(px)
        if d is not None:
            ds.append(d)
        time.sleep(max(0.0, delay))
    if not ds:
        return None, None
    ds.sort()
    median = ds[len(ds)//2]
    return ds[0], median  # (min, median)

def _servo_angle_for_left(angle_abs):
    return +angle_abs if SERVO_LEFT_POSITIVE else -angle_abs
def _servo_angle_for_right(angle_abs):
    return -angle_abs if SERVO_LEFT_POSITIVE else +angle_abs

def line_error_from_grayscale(px, line_is_white=False):
    vals = px.get_grayscale_data()          # [L,C,R]
    vL, vC, vR = vals
    thr = (min(vals) + max(vals)) / 2.0
    if line_is_white:
        L_on = vL > thr; C_on = vC > thr; R_on = vR > thr
    else:
        L_on = vL < thr; C_on = vC < thr; R_on = vR < thr
    if L_on and C_on and R_on:
        state, err = 'cross', 0
    elif not (L_on or C_on or R_on):
        state, err = 'lost', 0
    elif C_on and not (L_on or R_on):
        state, err = 'straight', 0
    elif L_on and not R_on:
        state, err = 'left', -1
    elif R_on and not L_on:
        state, err = 'right', +1
    else:
        state = 'junction'; err = (+1 if R_on else -1)
    return (vals, (int(L_on),int(C_on),int(R_on)), state, err)


# ========= Init Robot + Cam =========
px = Picarx()
if LINE_REF:
    try: px.set_line_reference(LINE_REF)
    except Exception: pass
try: px.set_cliff_reference(LINE_CLIFF_REF)
except Exception: pass

px.set_dir_servo_angle(0)
px.set_cam_pan_angle(0)
px.set_cam_tilt_angle(0)

Vilib.camera_start(vflip=CAM_VFLIP, hflip=CAM_HFLIP)
Vilib.display(local=CAM_LOCAL, web=CAM_WEB)
time.sleep(1.0)

player = Player(start_volume=START_VOLUME)

# Einzelrad-API (Bias) optional
HAS_DIRECT_MOTOR = False
try:
    from robot_hat import Motor
    M_LEFT  = Motor(1)
    M_RIGHT = Motor(2)
    HAS_DIRECT_MOTOR = True
except Exception:
    print("[Info] robot_hat.Motor not available -> using px.forward/backward only")

# Zusatz-Servo auf PWM3
HAS_EXTRA_SERVO = False
try:
    from robot_hat import Servo as _Servo
    EXTRA_SERVO = _Servo(EXTRA_SERVO_CHANNEL)
    HAS_EXTRA_SERVO = True
    # Startposition: CLOSE
    init_close = (-EXTRA_SERVO_CLOSE_ANGLE if EXTRA_SERVO_INVERT else EXTRA_SERVO_CLOSE_ANGLE)
    try:
        EXTRA_SERVO.angle(init_close)
    except Exception:
        pass
except Exception as e:
    print("[Info] Extra servo (PWM3) not available:", e)


# ========= Controller & State Machine =========
class DualSensePiCar(Controller):
    MODE_MANUAL  = 0
    MODE_LINE    = 1
    MODE_PARKOUR = 2

    def __init__(self, **kw):
        super().__init__(**kw)
        self.mode = self.MODE_MANUAL
        self.line_is_white = False

        # manual
        self.steer = 0.0
        self.fwd_pressed = False   # R2
        self.rev_pressed = False   # L2
        self.ignore_until = 0.0

        # toggles
        self.cliff_enabled = False
        self.obstacle_enabled = False

        # camera
        self.cam_pan_deg  = 0
        self.cam_tilt_deg = 0

        # extra servo state
        self.grip_open = False

        # line/parkour helpers
        self.last_on_line_ts = time.time()
        self.last_state = None

        # obstacle/cliff states
        self._obs_waiting = False
        self._cliff_active = False

        # cliff debounce & adaptive baseline
        self._cliff_true = 0
        self._cliff_false = 0
        self._cliff_cooldown_until = 0.0
        self._cliff_base = self._calibrate_cliff_base()
        self._cliff_alpha = CLIFF_ALPHA_BASE
        self._cliff_ratio_thr = CLIFF_RATIO_THR

        # worker
        self._stop_flag = False
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

        print("[Mode] MANUAL ready")

    # ----- Cliff: Baseline & Raw-Check -----
    def _calibrate_cliff_base(self, samples=20, settle=0.015):
        buf = []
        for _ in range(samples):
            try:
                buf.append(px.get_grayscale_data())
            except Exception:
                pass
            time.sleep(settle)
        if not buf:
            return [1500,1500,1500]
        ch = list(zip(*buf))
        base = [sorted(c)[len(c)//2] for c in ch]   # Median pro Kanal
        base = [max(b, 400) for b in base]          # Mindestbaseline
        print("[Cliff] baseline =", base)
        return base

    def _read_cliff_raw(self):
        """Cliff, wenn alle drei Sensoren deutlich unter Baseline (Verhältnis < Schwelle)."""
        vals = px.get_grayscale_data()
        ratios = [ v / (b if b>1 else 1) for v,b in zip(vals, self._cliff_base) ]
        is_cliff_now = (ratios[0] < self._cliff_ratio_thr and
                        ratios[1] < self._cliff_ratio_thr and
                        ratios[2] < self._cliff_ratio_thr)
        # Baseline nur updaten, wenn sicher kein Cliff
        if not is_cliff_now:
            a = self._cliff_alpha
            self._cliff_base = [ int((1-a)*b + a*v) for v,b in zip(vals, self._cliff_base) ]
        return is_cliff_now

    # ----- helpers -----
    def _now(self): return time.time()
    def _locked(self): return self._now() < self.ignore_until
    def _manual_allowed(self): return (self.mode == self.MODE_MANUAL) and not self._locked()

    def _set_servo(self):
        base = int(self.steer * STEER_MAX_DEG)
        if not SERVO_LEFT_POSITIVE:
            base = -base
        trim = 0 if HAS_DIRECT_MOTOR else int(STEER_TRIM_DEG)
        px.set_dir_servo_angle(base + trim)

    def _drive_biased(self, speed_signed):
        """Signed speed -100..100; Motor-Skalen separat für vorwärts/rückwärts je Seite."""
        sp = max(-100, min(100, int(speed_signed)))
        if sp == 0:
            px.stop(); return
        if HAS_DIRECT_MOTOR:
            if sp > 0:
                l = int(sp * MOTOR_L_FWD); r = int(sp * MOTOR_R_FWD)
            else:
                l = int(sp * MOTOR_L_REV); r = int(sp * MOTOR_R_REV)
            try:
                M_LEFT.speed(l); M_RIGHT.speed(r)
            except Exception:
                if sp > 0: px.forward(abs(sp))
                else:       px.backward(abs(sp))
        else:
            if sp > 0: px.forward(abs(sp))
            else:       px.backward(abs(sp))

    def _perform_u_turn(self, direction_right=True):
        px.stop()
        turn = _servo_angle_for_right(UTURN_STEER) if direction_right else _servo_angle_for_left(UTURN_STEER)
        px.set_dir_servo_angle(turn)
        self._drive_biased(+AUTO_SPEED)
        time.sleep(UTURN_TIME)
        px.stop()
        px.set_dir_servo_angle(0)

    def _handle_dead_end(self):
        print("[Parkour] Dead-End -> backup + U-Turn")
        px.set_dir_servo_angle(0)
        self._drive_biased(-CLIFF_BACKUP_SPEED)
        time.sleep(0.5)
        px.stop()
        self._perform_u_turn(direction_right=PARKOUR_RIGHT_HAND)

    def _set_extra_servo(self, open_state: bool):
        if not HAS_EXTRA_SERVO:
            print("[PWM3] servo not available")
            return
        ang_open  = (-EXTRA_SERVO_OPEN_ANGLE  if EXTRA_SERVO_INVERT else EXTRA_SERVO_OPEN_ANGLE)
        ang_close = (-EXTRA_SERVO_CLOSE_ANGLE if EXTRA_SERVO_INVERT else EXTRA_SERVO_CLOSE_ANGLE)
        try:
            EXTRA_SERVO.angle(ang_open if open_state else ang_close)
        except Exception as e:
            print("[PWM3] servo error:", e)

    # ----- main loop -----
    def _loop(self):
        while not self._stop_flag:
            try:
                # -------- Hindernisse (schnell) --------
                min_d = med_d = None
                if self.obstacle_enabled:
                    min_d, med_d = fast_ultra_burst(px)

                if self.mode in (self.MODE_LINE, self.MODE_PARKOUR):
                    # Hard stop bei sehr kleinem min-Abstand
                    if (min_d is not None) and (min_d < OBSTACLE_EMERGENCY_DISTANCE):
                        if not self._obs_waiting:
                            print(f"[Obstacle] EMERGENCY STOP ({min_d:.1f} cm)")
                            self._obs_waiting = True
                        px.stop()
                    # Normaler Stop bei median < Schwelle
                    elif (med_d is not None) and (med_d < OBSTACLE_STOP_DISTANCE):
                        if not self._obs_waiting:
                            print(f"[Obstacle] stop & wait ({med_d:.1f} cm)")
                            self._obs_waiting = True
                        px.stop()
                    else:
                        # prüfen ob wieder frei
                        if self._obs_waiting:
                            _min, _med = fast_ultra_burst(px)
                            if (_med or 0) >= (OBSTACLE_STOP_DISTANCE + OBSTACLE_RELEASE_HYST):
                                print("[Obstacle] clear -> continue")
                                self._obs_waiting = False
                else:
                    # MANUAL: KEIN Backup, wenn der Fahrer bereits rückwärts hält
                    if (min_d is not None) and (min_d < OBSTACLE_STOP_DISTANCE) and not self._locked():
                        if not self.rev_pressed:
                            print("[Obstacle] manual: {min_d:.1f} cm -> short backup")
                            px.set_dir_servo_angle(0)
                            self._drive_biased(-min(60, MANUAL_SPEED))
                            time.sleep(MANUAL_BACKUP_TIME)
                            px.stop()

                # -------- Cliff handling (debounced + adaptive) --------
                cliff = False
                if self.cliff_enabled:
                    raw = self._read_cliff_raw()
                    if raw:
                        self._cliff_true += 1; self._cliff_false = 0
                    else:
                        self._cliff_false += 1
                        if self._cliff_false >= CLIFF_CLEAR_COUNT:
                            self._cliff_true = 0
                            self._cliff_active = False
                    if (self._cliff_true >= CLIFF_DEBOUNCE_COUNT) and (self._now() >= self._cliff_cooldown_until):
                        cliff = True
                        self._cliff_active = True
                        self._cliff_cooldown_until = self._now() + CLIFF_COOLDOWN_TIME

                # -------- Modes --------
                if self.mode == self.MODE_MANUAL:
                    if cliff and not self._locked():
                        print("[Cliff] manual: edge -> backup & lock inputs")
                        px.set_dir_servo_angle(0)
                        self._drive_biased(-CLIFF_BACKUP_SPEED)
                        time.sleep(CLIFF_BACKUP_TIME)
                        px.stop()
                        self.ignore_until = self._now() + INPUT_LOCK_TIME
                    else:
                        # kontinuierlich den gehaltenen Trigger anwenden
                        if not self._locked():
                            self._apply_drive()

                elif self.mode == self.MODE_LINE:
                    vals, raw, st, err = line_error_from_grayscale(px, self.line_is_white)
                    if cliff:
                        print("[Cliff] line: STOP"); px.stop()
                    elif not self._obs_waiting:
                        if   st == 'straight' or err == 0:
                            px.set_dir_servo_angle(0);                 self._drive_biased(+AUTO_SPEED)
                        elif err < 0:
                            px.set_dir_servo_angle(_servo_angle_for_left(LINE_STEER_OFFSET));  self._drive_biased(+AUTO_SPEED)
                        elif err > 0:
                            px.set_dir_servo_angle(_servo_angle_for_right(LINE_STEER_OFFSET)); self._drive_biased(+AUTO_SPEED)
                        if st in ('cross','junction') and st != self.last_state:
                            print("[Line] junction/cross  raw=", raw, " vals=", [int(v) for v in vals])
                    self.last_state = st

                elif self.mode == self.MODE_PARKOUR:
                    vals, raw, st, err = line_error_from_grayscale(px, self.line_is_white)
                    if st != 'lost': self.last_on_line_ts = time.time()
                    if cliff:
                        print("[Cliff] parkour: edge -> backup & U-Turn")
                        px.set_dir_servo_angle(0)
                        self._drive_biased(-CLIFF_BACKUP_SPEED)
                        time.sleep(CLIFF_BACKUP_TIME)
                        px.stop()
                        self._perform_u_turn(direction_right=PARKOUR_RIGHT_HAND)
                    elif not self._obs_waiting:
                        if   st == 'straight' or err == 0:
                            px.set_dir_servo_angle(0);                  self._drive_biased(+AUTO_SPEED)
                        elif err < 0:
                            px.set_dir_servo_angle(_servo_angle_for_left(LINE_STEER_OFFSET));  self._drive_biased(+AUTO_SPEED)
                        elif err > 0:
                            px.set_dir_servo_angle(_servo_angle_for_right(LINE_STEER_OFFSET)); self._drive_biased(+AUTO_SPEED)
                        elif st in ('cross','junction'):
                            if self.last_state != st:
                                print("[Parkour] junction -> prefer", "RIGHT" if PARKOUR_RIGHT_HAND else "LEFT")
                            angle = _servo_angle_for_right(LINE_STEER_OFFSET) if PARKOUR_RIGHT_HAND else _servo_angle_for_left(LINE_STEER_OFFSET)
                            px.set_dir_servo_angle(angle); self._drive_biased(+AUTO_SPEED)
                        elif st == 'lost':
                            if (time.time() - self.last_on_line_ts) > DEADEND_LOST_TIME:
                                self._handle_dead_end()
                            else:
                                px.set_dir_servo_angle(0); self._drive_biased(+int(AUTO_SPEED*0.6))
                        else:
                            px.stop()
                    self.last_state = st

            except Exception as e:
                print("[Loop] exception:", e)
                time.sleep(0.2)

            time.sleep(MAIN_LOOP_SLEEP)

    # ========= Input-Events =========
    def on_up_arrow_press(self):
        self.mode = self.MODE_MANUAL; px.stop()
        print("[Mode] MANUAL")
    def on_right_arrow_press(self):
        self.mode = self.MODE_LINE;   px.stop()
        print("[Mode] LINE (L1 toggles WHITE/BLACK)")
    def on_share_press(self):
        self.mode = self.MODE_PARKOUR; px.stop(); self.last_on_line_ts = time.time()
        print("[Mode] PARKOUR")
    def on_down_arrow_press(self):
        self.cliff_enabled = not self.cliff_enabled
        print("[Cliff]", "ON" if self.cliff_enabled else "OFF")
        if self.cliff_enabled:
            self._cliff_base = self._calibrate_cliff_base()
    def on_left_arrow_press(self):
        self.obstacle_enabled = not self.obstacle_enabled
        print("[Obstacle]", "ON" if self.obstacle_enabled else "OFF")

    # L1: White/Black toggle (LINE & PARKOUR)
    def on_L1_press(self):
        if self.mode in (self.MODE_LINE, self.MODE_PARKOUR):
            self.line_is_white = not self.line_is_white
            print("[Line color]", "WHITE" if self.line_is_white else "BLACK")

    # R1: Audio-Stop
    def on_R1_press(self):
        player.stop()
        print("[Audio] stop")

    # Lenkung nur MANUAL
    def on_L3_left(self, v):   self._update_steer(v)
    def on_L3_right(self, v):  self._update_steer(v)
    def on_L3_x_at_rest(self): self._update_steer(0)
    def on_L3_up(self, v):     pass
    def on_L3_down(self, v):   pass
    def on_L3_y_at_rest(self): pass

    def _update_steer(self, x_raw):
        if not self._manual_allowed(): return
        self.steer = scale_axis(x_raw)
        self._set_servo()

    # Fahren MANUAL auf R2/L2 (max speed)
    def _apply_drive(self):
        if not self._manual_allowed(): return
        self._set_servo()
        if self.fwd_pressed and not self.rev_pressed:
            self._drive_biased(+MANUAL_SPEED)
        elif self.rev_pressed and not self.fwd_pressed:
            self._drive_biased(-MANUAL_SPEED)
        else:
            px.stop()

    def on_R2_press(self, value):  # vorwärts
        self.fwd_pressed = True
        self.rev_pressed = False   # exklusiv!
        self._apply_drive()
    def on_R2_release(self):
        self.fwd_pressed = False
        self._apply_drive()
    def on_L2_press(self, value):  # rückwärts
        self.rev_pressed = True
        self.fwd_pressed = False   # exklusiv!
        self._apply_drive()
    def on_L2_release(self):
        self.rev_pressed = False
        self._apply_drive()

    # Kamera: R3
    def on_R3_left(self, v):    self._cam_pan_update(-abs(v))
    def on_R3_right(self, v):   self._cam_pan_update(+abs(v))
    def on_R3_x_at_rest(self):  pass
    def _cam_pan_update(self, v):
        pan = CAM_PAN_INVERT * scale_axis(v)
        self.cam_pan_deg = int(max(-CAM_PAN_MAX, min(CAM_PAN_MAX, pan * CAM_PAN_MAX)))
        px.set_cam_pan_angle(self.cam_pan_deg)
    def on_R3_up(self, v):      self._cam_tilt_update(-v)
    def on_R3_down(self, v):    self._cam_tilt_update(-v)
    def on_R3_y_at_rest(self):  pass
    def _cam_tilt_update(self, v):
        tilt = CAM_TILT_INVERT * scale_axis(v)
        self.cam_tilt_deg = int(max(-CAM_TILT_MAX, min(CAM_TILT_MAX, tilt * CAM_TILT_MAX)))
        px.set_cam_tilt_angle(self.cam_tilt_deg)

    # Audio-Play (X/□), Kreis = Servo, △ = Bayern 1
    def on_x_press(self):        player.play_file(SOUND_X_PATH)
    def on_square_press(self):   player.play_file(SOUND_SQUARE_PATH)
    def on_circle_press(self):   # <<< Servo toggle auf PWM3
        self.grip_open = not self.grip_open
        self._set_extra_servo(self.grip_open)
        print("[PWM3] servo", "OPEN" if self.grip_open else "CLOSE")
    def on_triangle_press(self): player.play_url(BAYERN1_URL)
    def on_R3_press(self):       player.stop()

    # Not-Aus
    def on_options_press(self):
        self.fwd_pressed=False; self.rev_pressed=False
        self.ignore_until = 0.0
        self._obs_waiting = False
        self._cliff_active = False
        self.mode = self.MODE_MANUAL
        player.stop(); px.stop()
        px.set_dir_servo_angle(0)
        self.cam_pan_deg=0; self.cam_tilt_deg=0
        px.set_cam_pan_angle(0); px.set_cam_tilt_angle(0)
        # Servo optional wieder schließen:
        if HAS_EXTRA_SERVO:
            try: self._set_extra_servo(False)
            except Exception: pass
        print("[SAFE] STOP (reset to MANUAL)")

    def close(self):
        self._stop_flag = True
        try: self._th.join(timeout=0.5)
        except Exception: pass


if __name__ == "__main__":
    print("Start: DualSense -> PiCar-X (R2 vor / L2 rück, ○ PWM3 Servo, R1 Audio-Stop, L1 Black/White)")
    ctrl = DualSensePiCar(interface=JS_DEVICE, connecting_using_ds4drv=False)
    try:
        ctrl.listen(timeout=60)
    finally:
        try:
            ctrl.close(); player.stop()
        except Exception:
            pass
        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
        Vilib.camera_close()
        print("Beendet.")
