#!/usr/bin/env python3

import numpy as np
from class_uwb import UWB

def main():
    uwb = UWB(portName='/dev/ttyUSB1', topic='/makara_00/uwb', rate=5)
    uwb.start_uwb()


if __name__ == "__main__":
    main()
