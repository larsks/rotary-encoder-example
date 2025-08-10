DEVICE ?= /dev/ttyACM0
BOARD ?= rp2040:rp2040:waveshare_rp2040_zero:usbstack=tinyusb

compile:
	arduino-cli compile -b $(BOARD)

upload:
	arduino-cli upload -b $(BOARD) -p $(DEVICE)

connect:
	picocom -b 115200 $(DEVICE)
