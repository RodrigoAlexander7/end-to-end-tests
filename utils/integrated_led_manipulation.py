import os
import time


class PiIntegratedLED:
    """
    Controls the integrated LED on a Raspberry Pi (e.g., Pi Zero 2W).

    On most modern Raspberry Pi OS iterations, the integrated activity LED
    is exposed via the sysfs interface at /sys/class/leds/led0/.

    Note: Controlling the LED via sysfs typically requires root privileges
    (running your script with sudo) unless appropriate udev rules are set.
    """

    def __init__(self, led_name="led0"):
        """
        Initialize the LED controller.
        'led0' is typically the green activity LED.
        'led1' is typically the red power LED (not available on Pi Zero models).
        Sometimes the activity LED is named 'ACT' on newer kernels.
        """
        self.led_path = f"/sys/class/leds/{led_name}"
        self.brightness_path = os.path.join(self.led_path, "brightness")
        self.trigger_path = os.path.join(self.led_path, "trigger")
        self.max_brightness_path = os.path.join(self.led_path, "max_brightness")

        if not os.path.exists(self.brightness_path):
            alternate_path = "/sys/class/leds/ACT"
            if os.path.exists(alternate_path):
                print(
                    f"Warning: {self.led_path} not found. Using {alternate_path} instead."
                )
                self.led_path = alternate_path
                self.brightness_path = os.path.join(self.led_path, "brightness")
                self.trigger_path = os.path.join(self.led_path, "trigger")
                self.max_brightness_path = os.path.join(self.led_path, "max_brightness")
            else:
                print(
                    f"Error: Could not find LED control paths at {self.led_path} or {alternate_path}."
                )

        # Determine max brightness (usually 255 on a Pi Zero, not 1)
        self.max_brightness = 255
        if os.path.exists(self.max_brightness_path):
            try:
                with open(self.max_brightness_path, "r") as f:
                    self.max_brightness = int(f.read().strip())
            except Exception:
                pass

        # Read the current trigger so we can restore it later
        self.original_trigger = self.get_current_trigger()

    def get_current_trigger(self):
        if not os.path.exists(self.trigger_path):
            return "none"
        try:
            with open(self.trigger_path, "r") as f:
                content = f.read().strip()
                # content looks like: none [mmc0] timer default-on heartbeat
                for t in content.split():
                    if t.startswith("[") and t.endswith("]"):
                        return t[1:-1]
        except Exception:
            pass
        return "none"

    def set_trigger(self, trigger="none"):
        """
        Sets the trigger that controls the LED.
        To control it manually, we must set the trigger to 'none'.
        To return it to its default SD card activity behavior, set to 'mmc0'.
        """
        if not os.path.exists(self.trigger_path):
            return

        try:
            with open(self.trigger_path, "w") as f:
                f.write(trigger)
        except PermissionError:
            print(
                f"PermissionError: Cannot write to {self.trigger_path}. Try running as root (sudo)."
            )
        except Exception as e:
            print(f"Error setting trigger: {e}")

    def on(self):
        """Turn the LED on."""
        self._set_brightness(self.max_brightness)

    def off(self):
        """Turn the LED off."""
        self._set_brightness(0)

    def _set_brightness(self, value):
        if not os.path.exists(self.brightness_path):
            return

        try:
            with open(self.brightness_path, "w") as f:
                f.write(str(value))
        except PermissionError:
            print(
                f"PermissionError: Cannot write to {self.brightness_path}. Try running as root (sudo)."
            )
        except Exception as e:
            print(f"Error setting brightness: {e}")

    def blink(self, on_time=0.5, off_time=0.5, times=5):
        """Blink the LED a specific number of times."""
        self.set_trigger(
            "none"
        )  # Default trigger is usually mmc0, we need 'none' to control it manually

        for _ in range(times):
            self.on()
            time.sleep(on_time)
            self.off()
            time.sleep(off_time)


# --- Alternative: using gpiozero (if installed) ---
# If you prefer using the gpiozero library which is standard on Raspberry Pi,
# you can control the integrated LED like this:
"""
from gpiozero import LED
# For Pi Zero W / Pi Zero 2W, the ACT LED is often exposed as a specific pin or via the 'led0' name.
# Using 'led0' is more portable but requires the user to be in the 'gpio' group.

def blink_with_gpiozero():
    try:
        led = LED("led0") # Or LED(47) on older kernels
        led.blink(on_time=0.5, off_time=0.5, n=5, background=False)
    except Exception as e:
        print(f"gpiozero Error: {e}")
"""

if __name__ == "__main__":
    # Test the LED functionality
    print("Testing Pi Integrated LED...")
    led = PiIntegratedLED()

    print("Taking manual control over the LED...")
    led.set_trigger("none")

    print("Turning LED ON for 2 seconds...")
    led.on()
    time.sleep(2)

    print("Turning LED OFF for 2 seconds...")
    led.off()
    time.sleep(2)

    print("Blinking LED 5 times rapidly...")
    led.blink(on_time=0.1, off_time=0.1, times=5)

    print("Turning LED OFF for 2 seconds...")
    led.off()
    time.sleep(2)

    print("Returning LED to previous state (probably 'default-on' or 'mmc0')...")
    # If the user's LED was stuck in mmc0 due to a previous run of this script,
    # we can explicitly force it back to default-on if preferred:
    # led.set_trigger("default-on")
    led.set_trigger(led.original_trigger)

    # Just in case the previous state was none, make sure it stays ON if that was the usual
    if led.original_trigger == "none":
        led.on()

    print("Done.")
