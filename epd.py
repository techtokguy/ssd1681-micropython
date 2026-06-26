"""
MicroPython driver for GDEH0154D67 (1.54" 200x200 SSD1681) based on GxEPD2 Arduino sequences.
Target: ESP32 (adjust SPI init pins for other boards)

Wiring (example):
- CS  -> any GPIO (active low)
- DC  -> any GPIO
- RST -> any GPIO
- BUSY-> any GPIO (input)
- SCK -> HSPI SCK
- MOSI-> HSPI MOSI
- MISO-> optional (not used)

Notes:
- Panel requires 3.3V for VCC and IO. Do NOT use 5V data lines.
- Busy polarity for this variant: HIGH when busy.

Usage:
  import machine, time
  from micropython_gdeh0154d67 import EPD
  epd = EPD(spi_id=1, sck=14, mosi=13, cs=15, dc=27, rst=26, busy=25)
  epd.init()
  buf = bytearray([0xFF]*(200*200//8))  # white
  epd.write_image(0,0,200,200,buf)
  epd.update_full()
  epd.sleep()

"""

import time
from machine import Pin, SPI
import framebuf

# Display parameters
WIDTH = 200
HEIGHT = 200
BUSY_ACTIVE_LEVEL = 1  # busy HIGH for SSD1681

LAST_HASH = None  # global to track last image hash

class EPD:
    def __init__(self, spi_id=1, sck=14, mosi=13, miso=-1, baudrate=4000000, cs=5, dc=17, rst=16, busy=4):
        self.cs = Pin(cs, Pin.OUT, value=1)
        self.dc = Pin(dc, Pin.OUT, value=1)
        self.rst = Pin(rst, Pin.OUT, value=1)
        self.busy = Pin(busy, Pin.IN)
        self.spi = SPI(spi_id, baudrate=1000000, polarity=0, phase=0, sck=Pin(sck), mosi=Pin(mosi))  # miso not used by display

        # internal
        self._inited = False

    # low-level helpers
    def _cs_low(self):
        self.cs.value(0)
    def _cs_high(self):
        self.cs.value(1)
    def _dc_command(self):
        self.dc.value(0)
    def _dc_data(self):
        self.dc.value(1)

    def send_command(self, cmd):
        self._dc_command()
        self._cs_low()
        self.spi.write(bytes([cmd]))
        self._cs_high()
        self._dc_data()

    def send_data(self, data):
        # data can be int or bytes/bytearray
        self._dc_data()
        self._cs_low()
        if isinstance(data, int):
            self.spi.write(bytes([data]))
        else:
            # assume bytes-like
            self.spi.write(data)
        self._cs_high()

    def _start_data(self):
        self._dc_data()
        self._cs_low()

    def _end_data(self):
        self._cs_high()

    def reset(self):
        # follow Waveshare style: drive RST high, then low, then high
        print("Resetting EPD")
        self.rst.value(1)
        time.sleep_ms(10)
        self.rst.value(0)
        time.sleep_ms(10)
        self.rst.value(1)
        time.sleep_ms(10)
        print("Did reset, EPD")

    def wait_while_busy(self, timeout_ms=10000):
        start = time.ticks_ms()
        while self.busy.value() == BUSY_ACTIVE_LEVEL:
            time.sleep_ms(1)
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                raise OSError('EPD busy timeout')

    # partial ram area like in Arduino driver
    def set_partial_ram_area(self, x, y, w, h):
        # _writeCommand(0x11); _writeData(0x03);
        self.send_command(0x11)
        self.send_data(0x03)
        # 0x44 x start/end in bytes
        self.send_command(0x44)
        self.send_data(x // 8)
        self.send_data((x + w - 1) // 8)
        # 0x45 y start/end
        self.send_command(0x45)
        self.send_data(y & 0xFF)
        self.send_data((y >> 8) & 0xFF)
        yend = y + h - 1
        self.send_data(yend & 0xFF)
        self.send_data((yend >> 8) & 0xFF)
        # 0x4E ram x address
        self.send_command(0x4E)
        self.send_data(x // 8)
        # 0x4F ram y address
        self.send_command(0x4F)
        self.send_data(y & 0xFF)
        self.send_data((y >> 8) & 0xFF)

    # init sequence mirrored from _InitDisplay() in GxEPD2_154_D67.cpp
    def init(self):
        if self._inited:
            return
        self.reset()
        time.sleep_ms(10)
        # soft reset
        self.send_command(0x12)
        time.sleep_ms(10)
        # Driver output control
        self.send_command(0x01)
        self.send_data(0xC7)
        self.send_data(0x00)
        self.send_data(0x00)
        # Border Waveform
        self.send_command(0x3C)
        self.send_data(0x05)
        # Read built-in temp sensor
        self.send_command(0x18)
        self.send_data(0x80)
        # set full ram area
        self.set_partial_ram_area(0, 0, WIDTH, HEIGHT)
        self._inited = True
        print("EPD initialized")

    # power on sequence (PowerOn in Arduino code)
    def power_on(self):
        # _writeCommand(0x22); _writeData(0xe0); _writeCommand(0x20); _waitWhileBusy
        self.send_command(0x22)
        self.send_data(0xE0)
        self.send_command(0x20)
        self.wait_while_busy(5000)
        print("EPD powered on")

    def power_off(self):
        # _writeCommand(0x22); _writeData(0x83); _writeCommand(0x20); _waitWhileBusy
        self.send_command(0x22)
        self.send_data(0x83)
        self.send_command(0x20)
        self.wait_while_busy(2000)

    # write whole buffer (current) to RAM at specified rectangle and optionally do not refresh
    def write_image(self, x, y, w, h, buf):
        if not self._inited:
            self.init()
        # initial write handling in Arduino ensures previous/full buffers are managed; here we just write current
        self.set_partial_ram_area(x, y, w, h)
        self.send_command(0x24)  # write RAM (current)
        # stream data
        self._start_data()
        # buf should be bytes/bytearray length w*h/8
        self.spi.write(buf)
        self._end_data()

    # helper similar to Arduino's _Update_Full/_Update_Part
    def update_full(self):
        # full update: 0x22 0xF7, 0x20 then wait
        self.send_command(0x22)
        self.send_data(0xF7)
        self.send_command(0x20)
        # full refresh time in Arduino was relatively long; wait until busy releases
        self.wait_while_busy(20000)
        # after full update the Arduino sets power_is_on false; we keep state open for simplicity

    def update_partial(self):
        # partial update: 0x22 0xFC, 0x20 then wait
        self.send_command(0x22)
        self.send_data(0xFC)
        self.send_command(0x20)
        self.wait_while_busy(5000)

    def sleep(self):
        # deep sleep: 0x10 0x01 per Arduino
        self.power_off()
        self.send_command(0x10)
        self.send_data(0x01)

def connect_wifi():
    import network
    import time as utime
    import ntptime
    
    SSID=""
    PASS=""

    # connect Wi-Fi
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    sta.connect(SSID, PASS)
    while not sta.isconnected():
        print("Connecting to Wi-Fi...")
        utime.sleep(1)
    print(f"Connected! IP: {sta.ifconfig()[0]}")
    
    # Sync time with NTP server
    try:
        print("Syncing time with NTP...")
        ntptime.settime()
        print("Time synced successfully")
    except Exception as e:
        print(f"Warning: Could not sync time: {e}")
    
    return sta


def fetch_current_image(api_key, base_url="https://api.mistermatti.com/htmaa-final"):
    """
    Fetch the current image from the API if there's an update.

    This function is tasked with keeping track of hashes and making sure we never
    do any unecessary updating.
    
    Args:
        api_key: API key for authentication
        base_url: Base URL of the API
    
    Returns:
        tuple: (image_data, server_hash) where image_data is bytes or None
    """
    import urequests
    import ujson

    global LAST_HASH
    
    # Build URL with optional hash parameter
    url = f"{base_url}/image"
    if LAST_HASH is not None:
        url += f"?hash={LAST_HASH}"
    
    headers = {"X-API-Key": api_key}
    
    try:
        response = urequests.get(url, headers=headers)
        
        # Check if it's a JSON response (no update needed)
        content_type = response.headers.get("Content-Type", "")
        if "application/json" in content_type:
            data = ujson.loads(response.text)
            response.close()
            if data.get("updated") is False:
                print("No update needed (hash matches)")
                return None
            else:
                print(f"Error: {data.get('error', 'Unknown error')}")
                return None
        
        # Got binary image data
        if response.status_code == 200:
            # Get hash from response header
            server_hash = response.headers.get("X-Image-Hash")
            image_data = response.content
            response.close()

            LAST_HASH = server_hash  # update global last hash
            
            return image_data
        else:
            print(f"Error fetching image: {response.status_code}")
            response.close()
            return None
            
    except Exception as e:
        print(f"Error in fetch: {e}")
        return None


def update_display_from_api(api_key):
    """
    Fetch and display image from API if there's an update.
    
    Args:
        api_key: API key for authentication
        
    """
    import time
    
    # Fetch image (server will check hash and only return if different)
    image_data = fetch_current_image(api_key)
    
    if image_data is None:
        print("â­ï¸  Skipping - no update needed")
        return
    
    print(f"ðŸ”„ New image available")
    print(f"ðŸ“¥ Downloaded {len(image_data)} bytes")

    # Power on e-paper display
    enable_pin = Pin(20, Pin.OUT)
    enable_pin.value(0)  # turn on
    time.sleep_ms(100)
    
    epd = EPD(spi_id=1, sck=8, mosi=10, cs=5, dc=4, rst=3, busy=2)
    epd.init()
    epd.power_on()
    
    # Write image to display
    epd.write_image(0, 0, WIDTH, HEIGHT, image_data)
    epd.update_full()
    epd.sleep()
    
    # Turn off display power
    enable_pin.value(1)
    
    print(f"âœ… Display updated successfully")


# Simple test function to draw a checker pattern (for quick visible test)
def example_test():
    import time
    
    # Your API key - store this securely or load from config
    API_KEY = ""
    
    print("Starting WiFi connection.")
    sta = connect_wifi()
    print("WiFi connected.")
    
    print("\nðŸ” Starting update loop (checking every 4 seconds)...\n")
    
    while True:
        try:
            update_display_from_api(API_KEY)
            
            # Wait 4 seconds before next check
            time.sleep(4)
            
        except Exception as e:
            print(f"âŒ Error: {e}")
            time.sleep(4)  # Still wait before retrying

if __name__ == '__main__':
    example_test()
