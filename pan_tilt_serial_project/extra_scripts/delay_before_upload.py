Import("env")
import time

# Add delay before upload to allow auto-download circuit to activate
# This mimics the behavior of the Waveshare Flash Download Tool
def before_upload(source, target, env):
    print("Waiting for auto-download circuit to activate...")
    print("(This gives the ESP32 time to enter download mode)")
    time.sleep(1.0)  # 1 second delay for auto-download circuit

env.AddPreAction("upload", before_upload)
