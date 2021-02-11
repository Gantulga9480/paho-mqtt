import ctypes
import os

# Constants:
VERSION = '0.2.0'

# -------------------------------------------------------------------
# FUNCTIONS:
# -------------------------------------------------------------------
# OS-specific low-level timing functions:
if (os.name == 'nt'):  # for Windows:
    def micros():
        "return a timestamp in microseconds (us)"
        tics = ctypes.c_int64()
        freq = ctypes.c_int64()

        # get ticks on the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics))
        # get the actual freq. of the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))

        t_us = tics.value*1e6/freq.value
        return t_us

    def millis():
        "return a timestamp in milliseconds (ms)"
        tics = ctypes.c_int64()
        freq = ctypes.c_int64()

        # get ticks on the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics))
        # get the actual freq. of the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))

        t_ms = tics.value*1e3/freq.value
        return t_ms

elif (os.name == 'posix'):  # for Linux:

    # Constants:
    CLOCK_MONOTONIC_RAW = 4

    # prepare ctype timespec structure of {long, long}
    class timespec(ctypes.Structure):
        _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]

    librt = ctypes.CDLL('librt.so.1', use_errno=True)
    clock_gettime = librt.clock_gettime
    clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(timespec)]

    def monotonic_time():
        "return a timestamp in seconds (sec)"
        t = timespec()
        if clock_gettime(CLOCK_MONOTONIC_RAW, ctypes.pointer(t)) != 0:
            errno_ = ctypes.get_errno()
            raise OSError(errno_, os.strerror(errno_))
        return t.tv_sec + t.tv_nsec*1e-9

    def micros():
        "return a timestamp in microseconds (us)"
        return monotonic_time()*1e6

    def millis():
        "return a timestamp in milliseconds (ms)"
        return monotonic_time()*1e3


def delay(delay_ms):
    "delay for delay_ms milliseconds (ms)"
    t_start = millis()
    while (millis() - t_start < delay_ms):
        pass
    return


def delayMicroseconds(delay_us):
    "delay for delay_us microseconds (us)"
    t_start = micros()
    while (micros() - t_start < delay_us):
        pass
    return
