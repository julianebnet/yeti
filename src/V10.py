#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ASCII-only strings to avoid encoding issues.

from picarx import Picarx
from time import sleep
import readchar
import subprocess
import os
import time
import vlc
import time
from os import geteuid

manual = '''
Press keys on keyboard to control PiCar-X!
    w/a/s/d : Drive
    i/k/j/l : Head tilt / pan
    q       : Play/Stop audio
    + / -   : Player volume up / down (0..200)
    v       : Show player volume
    p       : Print audio status
    Ctrl+C  : Quit
'''

def show_info():
    print("\033[H\033[J", end="")
    print(manual)

# -------- Player with python-vlc first, subprocess fallback --------
class Player:

    # ---- API volume controls (0..200). No-op if fallback. ----
    def set_volume(self, vol):
        if not self.has_api:
            print("Player volume via API not available (using subprocess).")
            return None
        lvl = max(0, min(200, int(vol)))
        try:
            self._mp.audio_set_volume(lvl)
            return self.get_volume()
        except Exception:
            return None

    def get_volume(self):
        if not self.has_api:
            return None
        try:
            v = self._mp.audio_get_volume()
            # some libvlc return -1 on error
            return None if v < 0 else int(v)
        except Exception:
            return None

    def volume_up(self, step=10):
        cur = self.get_volume()
        if cur is None:
            return None
        return self.set_volume(cur + step)

    def volume_down(self, step=10):
        cur = self.get_volume()
        if cur is None:
            return None
        return self.set_volume(cur - step)

# --------------------------- main ---------------------------
if __name__ == "__main__":
    try:
        pan_angle = 0
        tilt_angle = 0
        px = Picarx()
        show_info()

        while True:
            key = readchar.readkey().lower()

            if key in ('w','s','a','d','i','k','j','l'):
                if key == 'w':
                    px.set_dir_servo_angle(0); px.forward(80)
                elif key == 's':
                    px.set_dir_servo_angle(0); px.backward(80)
                elif key == 'a':
                    px.set_dir_servo_angle(-35); px.forward(80)
                elif key == 'd':
                    px.set_dir_servo_angle(35); px.forward(80)
                elif key == 'i':
                    tilt_angle = min(35, tilt_angle + 5)
                elif key == 'k':
                    tilt_angle = max(-35, tilt_angle - 5)
                elif key == 'l':
                    pan_angle  = min(35, pan_angle + 5)
                elif key == 'j':
                    pan_angle  = max(-35, pan_angle - 5)

                px.set_cam_tilt_angle(tilt_angle)
                px.set_cam_pan_angle(pan_angle)
                show_info()
                sleep(0.5)
                px.forward(0)

            elif key == 'q':
                player = vlc.Instance('--no-video', '--aout=alsa', '--quiet')
                URL = "https://dispatcher.rndfnk.com/br/br1/nbopf/mp3/mid"
                media = player.media_new(URL)
                player = player.media_player_new()
                player.set_media(media)
                player.play()
                player.audio_set_volume(200)
            elif key == '+':
                newv = player.volume_up(10)
                if newv is not None:
                    print(f"Volume: {newv}%")
                else:
                    print("Volume control not available in subprocess mode.")

            elif key == '-':
                newv = player.volume_down(10)
                if newv is not None:
                    print(f"Volume: {newv}%")
                else:
                    print("Volume control not available in subprocess mode.")

            elif key == 'v':
                cur = player.get_volume()
                print(f"Volume: {cur}%" if cur is not None else "Volume unknown.")

            elif key == 'p':
                print("Audio status:", "playing" if player.is_playing() else "stopped")

            elif key == readchar.key.CTRL_C:
                print("\nQuit")
                break

    finally:
        try:
            player.stop()
            px.set_cam_tilt_angle(0)
            px.set_cam_pan_angle(0)
            px.set_dir_servo_angle(0)
            px.stop()
        except Exception:
            pass
        sleep(0.2)
