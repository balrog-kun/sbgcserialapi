This is a python module definiting message syntax for the SimpleBGC32 serial API as used by the gimbal software and hardware from BaseCam Electronics.

It uses the construct module (pip3 `construct` or debian/ubuntu `python3-construct` package) to define the frame and payload contents of each message type in a way that the single definition allows both building and parsing of the messages.  Also adds some utilities for handling the input and output from/to a serial port in a program's main loop.  See https://github.com/balrog-kun/openbgc/blob/master/utils/param-tool.py for an example usage, or the example below.

The definitions here aim to follow the official doc at https://www.basecamelectronics.com/files/SimpleBGC_2_6_Serial_Protocol_Specification.pdf.  At the time of writing the last release is from Oct 28, 2025.  Due to the expressiveness of the construct definition style, this may the most complete representation of that doc in code, notwithstanding any bugs.

The boring work here was done by Claude Sonet with manual checking, refactoring, merging of the pieces and fixing up after light testing.

=== Example ===

from sbgcserialapi import cmd, frame, unit
import serial

# Invoking the builder correctly can be a little complicated, work is still needed to allow more unused elements to be skipped

out_payload = cmd.ControlRequest.build(dict(control_mode=(cmd.ControlMode.MODE_ANGLE, {}, {}), target=(dict(angle=int(10 / unit.degree_factor)), {}, {})))
out_frame = frame.FrameV1.build(dict(hdr=dict(cmd_id=int(cmd_obgc.CmdId.CMD_CONTROL), size=len(out_payload)), pld=out_payload))

sbgc_port = serial.Serial(sbgc_port_path, baudrate=115200, timeout=0, write_timeout=1)
sbgc_port.write(out_frame)
