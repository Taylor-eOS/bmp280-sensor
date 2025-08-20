#!/usr/bin/env python3
import csv
import time
import os
from smbus2 import SMBus
from datetime import datetime
from pathlib import Path
from read_bmp import read_cal, write_config, read_raw, compensate, find_addr

def main():
    out_path = Path("temperature_log.csv")
    new_file = not out_path.exists()
    with SMBus(1) as bus, open(out_path, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(["timestamp", "temperature_C", "pressure_hPa", "humidity_percent"])
        addr, cid = find_addr(bus)
        if not addr:
            print("No BME/BMP found at 0x76/0x77")
            return
        is_bme = (cid == 0x60)
        print("Logging from device at 0x%02X (%s)" % (addr, "BME280" if is_bme else "BMP280"))
        cal = read_cal(bus, addr)
        write_config(bus, addr)
        while True:
            try:
                raw_t, raw_p, raw_h = read_raw(bus, addr, is_bme)
                temp, press, hum = compensate(cal, raw_t, raw_p, raw_h)
                writer.writerow([datetime.now().isoformat(), f"{temp:.2f}", f"{press/100.0:.2f}", f"{hum:.2f}" if hum is not None else ""])
                f.flush()
                print("Logged: %.2f C, %.2f hPa, %s" % (temp, press/100.0, ("%.2f %%" % hum) if hum is not None else "N/A"))
            except Exception as e:
                print("Read error", e)
            time.sleep(60)

