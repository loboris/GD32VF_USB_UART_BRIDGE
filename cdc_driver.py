# =========================================
# GD32VF103 USB<>UART bridge example driver
# Copyright 2020 LoBo
# MIT License
# =========================================

import usb.core
import usb.util
import usb.control
import array
import sys, time
import threading
import atexit

# The receiver thread
# Data received from USB CDC are saved into the receive buffer
# ------------------------------------------------------------
def ReceiveData(shutdown, ep_read, rdbuf):
    print("Receive thread started.")
    try:
        while not shutdown.is_set():
            try:
                data = ep_read.read(100, 20)
                if len(data) > 0:
                    rdbuf.extend(data)
            except:
                continue
    except Exception as e:
        print("Receive thread ERROR\r\n{}".format(e))
    print("Receive thread terminated.")
# ------------------------------------------------------------

class cdcDriver:

    def __init__(self, baudrate=115200):
        self.VENDOR_GD32VF = 0x28e9
        self.PRODUCT_GD32VF = 0x018a
        self.INTERFACE_GD32VF = 0
        self.BAUDRATE = baudrate
        self.device = None
        self.ep_write = None
        self.ep_read = None
        self.interface = None
        self.rdbuf = array.array('B')
        self.ShutdownReceiver = threading.Event()
        self.ReceiverThread = None

        try:
            self.device = usb.core.find(idVendor=self.VENDOR_GD32VF, idProduct=self.PRODUCT_GD32VF)
            if self.device is None:
                raise Exception("GD32VF device not found")

            # By default, the kernel will claim the device and make it available via
            # /dev/usb/ACMn which also prevents us from communicating otherwise.
            # This removes these kernel devices.
            if self.device.is_kernel_driver_active(self.INTERFACE_GD32VF):
                print("Detaching kernel driver")
                self.device.detach_kernel_driver(self.INTERFACE_GD32VF)
            else:
                print("Kernel driver not attached")

            self.interface = usb.util.find_descriptor(self.device.get_active_configuration(),bInterfaceNumber=1)
            if self.interface is None:
                raise Exception("GD32VF interface not found")
            self.ep_write = self.interface[0]
            self.ep_read = self.interface[1]
            if self.ep_write is None or self.ep_read is None:
                raise Exception("GD32VF endpoints not found")

            # Start receiver thread
            self.ReceiverThread = threading.Thread(target=ReceiveData, name="CDC_Read", args=(self.ShutdownReceiver, self.ep_read, self.rdbuf), daemon=True)
            self.ReceiverThread.start()

            self.setFormat(bdr=self.BAUDRATE)

            atexit.register(self.stopReceive)
        except:
            raise Exception("Accessing GD32VF device failed")

    # --------------------------
    # Stop the receiver thread
    # Usually not used directly
    # -------------------------
    def stopReceive(self): 
        if self.ReceiverThread is None:
            print("Receive thread not started.")
            return True
        print("Terminate receive thread request.")
        self.ShutdownReceiver.set()
        if self.ReceiverThread.isAlive():
            self.ReceiverThread.join(1)
        if self.ReceiverThread.isAlive():
            self.ShutdownReceiver.clear()
            return False
        else:
            self.ShutdownReceiver.clear()
            self.ReceiverThread = None
            return True

    # ----------------------------------------------------
    # Start the receive thread if it was previously stoped
    # ----------------------------------------------------
    def startReceive(self): 
        if self.ReceiverThread is not None and self.ReceiverThread.isAlive():
            return True
        else:
            self.ShutdownReceiver.clear()
            self.ReceiverThread = threading.Thread(target=ReceiveData, name="CDC_Read", args=(self.ShutdownReceiver, self.ep_read, self.rdbuf), daemon=True)
            self.ReceiverThread.start()
            return True

    # -------------------------------------------------------
    # Show the current status on the device display (if used)
    # If the 'reset' argument id 1, reset the counters
    # -------------------------------------------------------
    def showStatus(self, reset = 0):
        try:
            res = self.device.ctrl_transfer(0x21, 0xE0, reset & 1, 0, None)
            return (res == 0)
        except:
            return False

    # -----------------------------------
    # Set DTR and RTS line state
    # -----------------------------------
    def setLineState(self, dtr=1, rts=1):
        try:
            res = self.device.ctrl_transfer(0x21, 0x22, (dtr & 1) | ((rts & 1) << 1), 0, None)
            return (res == 0)
        except:
            return False

    # ------------------------------------------------------
    # Set active device UART, 0 or 1 can be used as argument
    # ------------------------------------------------------
    def setUART(self, uartno=0):
        try:
            res = self.device.ctrl_transfer(0x21, 0xE1, uartno, 0, None)
            return (res == 0)
        except:
            return False

    # -----------------------------------------------------------
    # Set the UART communication format
    #    bdr: baudrate, 2400 - 4000000
    #   bits: 8 or 7
    # parity: 0 - None; 2 - Even, 3 - Odd
    #   stop: number of stop bits; 0 = 1, 1 = 0.5, 2 = 2, 3 = 1.5
    # -----------------------------------------------------------
    def setFormat(self, bdr=115200, bits=8, parity=0, stop=0):
        try:
            bdr_array = array.array('B', [bdr & 0xFF, (bdr >> 8) & 0xFF, (bdr >> 16) & 0xFF, (bdr >> 24) & 0xFF, stop, parity, bits])
            res = self.device.ctrl_transfer(0x21, 0x20, 0, 0, bdr_array)
            return (res == 7)
        except:
            return False

    # -------------------------
    def data32(self, buf, idx):
        return buf[idx] + (buf[idx+1] << 8) + (buf[idx+2] << 16) + (buf[idx+3] << 24)

    # ----------------------------------------------------------
    # Get the current UART communication format from the device
    # If 'prn' argument is True, prints the format
    # Returns tuple: (bdr, stop, parity, bits)
    # ----------------------------------------------------------
    def getFormat(self, prn=False):
        try:
            res = self.device.ctrl_transfer(0xA1, 0x21, 0, 0, 7, 100)
            if len(res) == 7:
                bdr = self.data32(res, 0)
                tres = (bdr, res[4], res[5], res[6])
                if prn:
                    prty = '?'
                    if res[5] == 0:
                        prty = 'N'
                    elif res[5] == 2:
                        prty = 'E'
                    elif res[5] == 3:
                        prty = 'O'
                    sb = '?'
                    if res[4] == 0:
                        sb = '1'
                    elif res[4] == 1:
                        sb = '0.5'
                    elif res[4] == 2:
                        sb = '2'
                    elif res[4] == 3:
                        sb = '1.5'
                    print("UART Format: {} {}{}{}".format(bdr, res[6], prty, sb))
                return tres
            else:
                return False
        except:
            return False

    # -------------------------------------------------------------------------------------------------------------------------
    # Get the current transmition status
    # If 'prn' argument is True, prints the format
    # Returns tuple: (active_uart, dtr_state, rts_state, usb_sent, usb_received, uart_sent, uart_received, uart_receive_errors)
    # -------------------------------------------------------------------------------------------------------------------------
    def getStatus(self, prn=False):
        try:
            res = self.device.ctrl_transfer(0xA1, 0xE2, 0, 0, 22, 100)
            if len(res) == 22:
                send_count = self.data32(res, 2)
                receive_count = self.data32(res, 6)
                uart_send_count = self.data32(res, 10)
                uart_receive_count = self.data32(res, 14)
                uart_error_count = self.data32(res, 18)
                tres = (res[0], res[1] & 1, res[1] >> 1, send_count, receive_count, uart_send_count, uart_receive_count, uart_error_count)
                if prn:
                    print("Active UART: {}; DTR={}, RTS={}".format(tres[0], res[1] & 1, res[1] >> 1))
                    print("    USB CDC: sent {}, received {}".format(send_count, receive_count))
                    print("       UART: sent {}, received {}, errors {}".format(uart_send_count, uart_receive_count, uart_error_count))
                return tres
            else:
                return False
        except:
            return False

    # -------------------------------------------------
    # Return number of received bytes in receive buffer
    # -------------------------------------------------
    def received(self):
        return len(self.rdbuf)

    # ----------------------------------
    # Read data from receive buffer
    # count: max number of bytes to read
    # Returns array of bytes
    # ----------------------------------
    def read(self, count=1):
        if count < 1:
            return array.array('B')
        data = self.rdbuf[0:count]
        del self.rdbuf[:count]
        return data

    # -------------------------------------------------
    # Read data from receive buffer converted to string
    # count: max number of bytes to read
    # -------------------------------------------------
    def readString(self, count=1):
        if count < 1:
            return ""
        data = self.rdbuf[0:count]
        del self.rdbuf[:count]
        try:
            sdata = data.tostring().decode()
        except:
            sdata = ""        
        return sdata

    # --------------------
    # Write data
    # --------------------
    def write(self, data):
        try:
            res = 0
            if len(data) > 0:
                res = self.ep_write.write(data)
            return (res == len(data))
        except:
            return False



# === Usage examples ===

'''
from cdc_driver import cdcDriver
import time, threading

cdc = cdcDriver()

x = "A"*50000

#cdc.setFormat(bdr=4000000)

cdc.write("1234567890ABCDEFGHIJ1234567890ABCDEFGHIJ1234567890ABCDEFGHIJ1234567890ABCDEFGHIJ\n")

b = cdc.read(cdc.received())

s = cdc.readString(cdc.received())
print(s)

t1=time.time(); _=cdc.write(x[0:10000]); t2=time.time(); _=cdc.write("\n"); print(t2-t1, 10000/(t2-t1))
'''

