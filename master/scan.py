from time import sleep_ms
from machine import Pin
from onewire import OneWire
from ds18x20 import DS18X20

def scan_with_pause(ds, pause_ms=10):
    roms = []
    rom, diff = None, 65

    while True:
        rom, diff = ds.ow._search_rom(rom, diff)
        if rom is None:
            break
        roms.append(rom)
        if diff == 0:
            break
        sleep_ms(pause_ms)

    return roms

def init_ds(gpio: int):
    return DS18X20(OneWire(Pin(gpio)))

def scan_all_ds(ds):
    roms = scan_with_pause(ds)
    print(len(roms), [rom[1] for rom in roms])