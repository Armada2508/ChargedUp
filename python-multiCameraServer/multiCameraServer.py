#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import sys
import time
from typing import Final

import cv2
import numpy as np
from cscore import CameraServer, MjpegServer, UsbCamera, VideoSource
from ntcore import EventFlags, NetworkTable, NetworkTableInstance

#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)

    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break

    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)

    return server



###################################################################################################################


hueMin: Final[int] = 8
hueMax: Final[int] = 40
saturationMin: Final[int] = 150
saturationMax: Final[int] = 255
valueMin: Final[int] = 160
valueMax: Final[int] = 255
erodeIterations: Final[int] = 1
dilateIterations: Final[int] = 1


def main(): # Image proccessing user code
    CameraServer.enableLogging()
    cvSink = CameraServer.getVideo()
    outputStream = CameraServer.putVideo("Proccessed Video", 1280, 720)
    img = np.zeros(shape=(1280, 720, 3), dtype=np.uint8)
    nt: NetworkTable = NetworkTableInstance.getTable('vision')
    # loop forever
    while True:
        time, input_img = cvSink.grabFrame(img)

        if time == 0: # There is an error
            outputStream.notifyError(cvSink.getError());
            continue
        
        # Color thresholding
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        binary_img = cv2.inRange(hsv_img, (
            nt.getNumber("hueMin", hueMin),
            nt.getNumber("saturationMin", saturationMin) , 
            nt.getNumber("valueMin", valueMin)
        ), (
            nt.getNumber("hueMax", hueMax), 
            nt.getNumber("saturationMax", saturationMax), 
            nt.getNumber("valueMax", valueMax)
        ))
        
        # Eroding
        kernel = np.ones((3, 3), np.uint8)
        binary_img = cv2.erode(binary_img, kernel, iterations = nt.getNumber("erodeIterations", erodeIterations))
        
        # Dilating
        kernel = np.ones((3, 3), np.uint8)
        binary_img = cv2.dilate(binary_img, kernel, iterations = nt.getNumber("dilateIterations", dilateIterations))
        
        # Contours
        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            mainContour = contours[0]
            for contour in contours:
                if cv2.contourArea(contour) > cv2.contourArea(mainContour):
                    mainContour = contour
            # Center
            rect = cv2.minAreaRect(mainContour)
            center, size, angle = rect
            center = [int(dim) for dim in center] # Convert to int so we can draw
            # Corners
            # corners = cv2.convexHull(contour)
            # corners = cv2.approxPolyDP(corners, 0.1 * cv2.arcLength(contour), True)
            # # Rotation
            # _, _, rotation = cv2.fitEllipse(contour)
            # Convert to color to draw stuff
            binary_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
            binary_img = cv2.drawContours(binary_img, mainContour, -1, color = (0, 0, 255), thickness = 3)
            binary_img = cv2.circle(binary_img, center = center, radius = 10, color = (0, 255, 0), thickness = 2)
            outputStream.putFrame(binary_img)
            


###################################################################################################################



if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)
        
    main() # start user code
