import RPi.GPIO as GPIO
import time

# Define GPIO pin numbers (adjust based on your Raspberry Pi model and wiring)
CH_THROTTLE_IN = 20  # Replace with your desired PWM input pin (BCM numbering)
CH_STEERING_IN = 26  # Replace with your desired PWM input pin (BCM numbering)
NUM_CHANNELS = 2
PWM_DISCONNECT_THRESHOLD = 700  # Microsecond threshold for disconnection

# Define pin numbering scheme (BCM is recommended for clarity)
GPIO.setmode(GPIO.BCM)

# Set up input channels as inputs
for pin in [CH_THROTTLE_IN, CH_STEERING_IN]:
    GPIO.setup(pin, GPIO.IN)

def measure_pulse_width(channel):
    """
    Measures the pulse width of a PWM signal on the specified GPIO channel.

    Args:
        channel (int): The GPIO channel to measure the pulse width on.

    Returns:
        float: The pulse width in microseconds.
    """
    GPIO.wait_for_edge(channel, GPIO.RISING)  # Wait for rising edge
    start_time = time.time() * 1e6  # Convert to microseconds

    GPIO.wait_for_edge(channel, GPIO.FALLING)  # Wait for falling edge
    end_time = time.time() * 1e6  # Convert to microseconds

    pulse_width = end_time - start_time
    return pulse_width

def read_pwm_values():
    """
    Reads PWM values from the specified channels and checks for disconnection.

    Returns:
        list: A list containing the PWM values for each channel and a disconnec>
    """
    channel_values = [measure_pulse_width(CH_THROTTLE_IN), measure_pulse_width(>
    disconnection = any(value < PWM_DISCONNECT_THRESHOLD for value in channel_v>
    
    return channel_values, disconnection

if __name__ == "__main__":
    try:
        while True:
            # Read PWM values and disconnection status
            channel_values, is_disconnected = read_pwm_values()

            # Print data for debugging or further processing
            print("PWM Values:", channel_values)
            print("Disconnected:", is_disconnected)

            time.sleep(0.1)  # Adjust delay as needed
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        GPIO.cleanup()  # Clean up GPIO resources

