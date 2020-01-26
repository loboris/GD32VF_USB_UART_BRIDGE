<br>

## GD32VF103 USB to UART bridge

<br>

This project shows how to use **GD32VF103** board as USBtoUART bridge.

> The project is intended to be run on **Longan Nano** board.<br>
> If testing on another board (without a display), edit `lib/usbcdc/include/cdc_acm_core.h` to remove display and/or LED support.

### Features:

* The project implements USB CDC/ACM device with some additional features.
* Bridging USB to **two** UARTS is supported.
* Display support, displays status and statictics.
* Baudrates from 2400 ~ 4000000 Bd are supported
* Python CDC driver with examples included

### Pins used for UARTs:

**UART0**:
* `PA9  Tx`
* `PA10 Rx`
* `PA13 RTS`
* `PA14 DTR`

**UART1**:
* `PB10 Tx`
* `PB11 Rx`
* `PB13 RTS`
* `PB14 DTR`

### Details

After the board is connected to PC it will be enumerated as USB device `28e9:018a`.
```
Bus 003 Device 028: ID 28e9:018a  
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            2 Communications
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        64
  idVendor           0x28e9 
  idProduct          0x018a 
  bcdDevice            1.00
  iManufacturer           1 GigaDevice
  iProduct                2 GD32VF USB CDC ACM in FS Mode
  iSerial                 3 GD32VF-3.0.0-012020
  bNumConfigurations      1
...
```

The device can be accessed for serial communication as `/dev/ttyACMn`, for example via Minicom, Putty etc.
```
[24377.013615] usb 3-9: new full-speed USB device number 28 using xhci_hcd
[24377.167007] usb 3-9: New USB device found, idVendor=28e9, idProduct=018a, bcdDevice= 1.00
[24377.167011] usb 3-9: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[24377.167013] usb 3-9: Product: GD32VF USB CDC ACM in FS Mode
[24377.167015] usb 3-9: Manufacturer: GigaDevice
[24377.167017] usb 3-9: SerialNumber: GD32VF-3.0.0-012020
[24377.168506] cdc_acm 3-9:1.0: ttyACM0: USB ACM device
```

As the device has some non-standard CDC features, it is recommended to use it via the provided Python driver `cdc_driver.py`.

### Switching UARTS

GD32VF103 does not have enough USB endpoints to implement a composite CDC device.<br>
The UART ports can be changed using one of two methods:
* sending a `break` condition, this is usually used from standard communication program
* sending a special CDC command, this is fully supported and described in `cdc_driver.py`

### Notes

If _Longan Nano_ is used and `USE_LEDS` enabled in configuration, green LED will be active when `UART1` is selected.

If _Longan Nano_ is used and `USE_DISPLAY` enabled in configuration, UART format and DTR/RTS status will be shown on display.<br>
Special CDC command can be used to display the communication statistics (USB number of sent/received bytes, UART number of sent/received byted, number of UART receive errors,...).

DTR and RTS outputs can be configured as push-pull or open colector (`UARTn_RTS_DTR_MODE` define in `lib/usbcdc/include/cdc_acm_core.h`).

Using `break` for UART change can be disabled/enabled using `UART_USE_BREAK_TO_CHANGE` define in `lib/usbcdc/include/cdc_acm_core.h`.

<br>

### Functions supported in `cdc_driver.py`:
```python
# -------------------------------------------------------
# Show the current status on the device display (if used)
# If the 'reset' argument id 1, reset the counters
# -------------------------------------------------------
showStatus(reset = 0)

# -----------------------------------
# Set DTR and RTS line state
# -----------------------------------
setLineState(dtr=1, rts=1)

# ------------------------------------------------------
# Set active device UART, 0 or 1 can be used as argument
# ------------------------------------------------------
setUART(uartno=0)

# -----------------------------------------------------------
# Set the UART communication format
#    bdr: baudrate, 2400 - 4000000
#   bits: 8 or 7
# parity: 0 - None; 2 - Even, 3 - Odd
#   stop: number of stop bits; 0 = 1, 1 = 0.5, 2 = 2, 3 = 1.5
# -----------------------------------------------------------
setFormat(bdr=115200, bits=8, parity=0, stop=0)

# ----------------------------------------------------------
# Get the current UART communication format from the device
# If 'prn' argument is True, prints the format
# Returns tuple: (bdr, stop, parity, bits)
# ----------------------------------------------------------
getFormat(prn=False)

# --------------------------------------------
# Get the current transmition status
# If 'prn' argument is True, prints the format
# Returns tuple:
#   (
#    active_uart,
#    dtr_state,
#    rts_state,
#    usb_sent,
#    usb_received,
#    uart_sent,
#    uart_received,
#    uart_receive_errors
#   )
# --------------------------------------------
getStatus(prn=False)

# -------------------------------------------------
# Return number of received bytes in receive buffer
# -------------------------------------------------
received()

# ----------------------------------
# Read data from receive buffer
# count: max number of bytes to read
# Returns array of bytes
# ----------------------------------
read(count=1)

# -------------------------------------------------
# Read data from receive buffer converted to string
# count: max number of bytes to read
# -------------------------------------------------
readString(count=1)

# --------------------
# Write data
# --------------------
write(data)
```
