from binascii import hexlify
from time import sleep_ms

from scan import init_ds, scan_with_pause


ONEWIRE_GPIO = 12  # GPIO on the MicroPython master connected to D2 of the ESP8266.
SEARCH_PAUSE_MS = 50
CONVERSION_MS = 750
POLL_PAUSE_MS = 1000


def poll_temperatures(ds):
    roms = [rom for rom in scan_with_pause(ds, SEARCH_PAUSE_MS)
            if rom[0] == 0x28 and ds.ow.crc8(rom) == 0]
    print("DS18B20 found:", len(roms))
    if not roms:
        return

    ds.convert_temp()
    sleep_ms(CONVERSION_MS)
    for rom in roms:
        address = hexlify(rom).decode()
        try:
            temperature = ds.read_temp(rom)
        except Exception as error:
            # The driver reports a scratchpad CRC failure as a plain Exception.
            print("{}: read error: {}".format(address, error))
        else:
            print("{}: {:.2f} C".format(address, temperature))


def main():
    ds = init_ds(ONEWIRE_GPIO)
    print("1-Wire master on GPIO{}".format(ONEWIRE_GPIO))
    while True:
        try:
            poll_temperatures(ds)
        except Exception as error:
            print("1-Wire scan or conversion error:", error)
        sleep_ms(POLL_PAUSE_MS)


if __name__ == "__main__":
    main()
