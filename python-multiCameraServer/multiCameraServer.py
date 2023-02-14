#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import math
import sys
from typing import Final

import cv2
import numpy as np
from cscore import CameraServer, MjpegServer, UsbCamera, VideoSource
from ntcore import EventFlags, NetworkTable, NetworkTableInstance

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

class Pipeline:
    
    def __init__(self, hueMin: int, hueMax: int, saturationMin: int, saturationMax: int, valueMin: int, valueMax: int, erodeIterations: int, dilateIterations: int, pipelineIndex: int):
        self.hueMin: Final[int] = hueMin
        self.hueMax: Final[int] = hueMax
        self.saturationMin: Final[int] = saturationMin
        self.saturationMax: Final[int] = saturationMax
        self.valueMin: Final[int] = valueMin
        self.valueMax: Final[int] = valueMax
        self.erodeIterations: Final[int] = erodeIterations
        self.dilateIterations: Final[int] = dilateIterations
        self.pipelineIndex: Final[int] = pipelineIndex
        
# Camera
resolutionWidth: Final[int] = 1280
resolutionHeight: Final[int] = 720
verticalFOV: Final[int] = 0.644550094 # rad
horizontalFOV: Final[int] = 1.07145763 # rad

# Processing
currentPipeline: int = 0
conePipelineIndex: Final[int] = 0
cubePipelineIndex: Final[int] = 1
aprilTagsPipelineIndex: Final[int] = 2
conePipeline: Pipeline = Pipeline(8, 40, 120, 255, 160, 255, 1, 1, conePipelineIndex)
cubePipeline: Pipeline = Pipeline(120, 150, 30, 255, 120, 255, 1, 3, cubePipelineIndex)

# NetworkTables
networkTableName: Final[str] = "Vision"
nt: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)
hueMinNT: Final = nt.getIntegerTopic("hueMin").getEntry(0)
hueMaxNT: Final = nt.getIntegerTopic("hueMax").getEntry(0)
saturationMinNT: Final = nt.getIntegerTopic("saturationMin").getEntry(0)
saturationMaxNT: Final = nt.getIntegerTopic("saturationMax").getEntry(0)
valueMinNT: Final = nt.getIntegerTopic("valueMin").getEntry(0)
valueMaxNT: Final = nt.getIntegerTopic("valueMax").getEntry(0)
erodeIterationsNT: Final = nt.getIntegerTopic("erodeIterations").getEntry(0)
dilateIterationsNT: Final = nt.getIntegerTopic("dilateIterations").getEntry(0)
pipelineNT: Final = nt.getIntegerTopic("Pipeline").getEntry(0)
pitchNT: Final = nt.getFloatTopic("Pitch").getEntry(0)
yawNT: Final = nt.getFloatTopic("Yaw").getEntry(0)
haveTargetNT: Final = nt.getBooleanTopic("HaveTarget").getEntry(False)

def pointToPitchAndYaw(px: int, py: int): # Converts a point in pixel system to a pitch and a yaw and returns that.
    nx: float = (2/resolutionWidth) * (px - ((resolutionWidth/2) - 0.5))
    ny: float = (2/resolutionHeight) * (((resolutionHeight/2) - 0.5) - py)
    vpw: float = 2.0*math.tan(horizontalFOV/2)
    vph: float = 2.0*math.tan(verticalFOV/2)
    x: float = vpw/2 * nx
    y: float = vph/2 * ny
    ax: float = math.atan(x/1)
    ay: float = math.atan(y/1)
    yaw: float = math.degrees(ax)
    pitch: float = math.degrees(ay) * -1
    return (pitch, yaw)

def aprilTagPipeline():
    pass

def executePipeline(input_img, pipeline: Pipeline):
    global currentPipeline
    if (pipeline.pipelineIndex != currentPipeline):
        setupNetworkTables(pipeline)
    nt: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)
    # Color Converting
    hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
    # Color Thresholding
    binary_img = cv2.inRange(hsv_img, (
        hueMinNT.get(pipeline.hueMin),
        saturationMinNT.get(pipeline.saturationMin) , 
        valueMinNT.get(pipeline.valueMin)
    ), (
        hueMaxNT.get(pipeline.hueMax), 
        saturationMaxNT.get(pipeline.saturationMax), 
        valueMaxNT.get(pipeline.valueMax)
    ))
    # Eroding
    kernel = np.ones((3, 3), np.uint8)
    binary_img = cv2.erode(binary_img, kernel, iterations = int(erodeIterationsNT.get(pipeline.erodeIterations)))
    # Dilating
    kernel = np.ones((3, 3), np.uint8)
    binary_img = cv2.dilate(binary_img, kernel, iterations = int(dilateIterationsNT.get(pipeline.dilateIterations)))
    currentPipeline = pipeline.pipelineIndex
    return binary_img
        
def setupNetworkTables(pipeline: Pipeline):
    nt: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)
    hueMinNT.set(pipeline.hueMin)
    hueMaxNT.set(pipeline.hueMax)
    saturationMinNT.set(pipeline.saturationMin)
    saturationMaxNT.set(pipeline.saturationMax)
    valueMinNT.set(pipeline.valueMin)
    valueMaxNT.set(pipeline.valueMax)
    erodeIterationsNT.set(pipeline.erodeIterations)
    dilateIterationsNT.set(pipeline.dilateIterations)
    pipelineNT.set(pipeline.pipelineIndex)
    pitchNT.set(0)
    yawNT.set(0)
    haveTargetNT.set(False)

def main(): # Image proccessing user code
    CameraServer.enableLogging()
    cvSink = CameraServer.getVideo()
    outputStream = CameraServer.putVideo("Proccessed Video", resolutionWidth, resolutionHeight)
    originalStream = CameraServer.putVideo("Original Video", resolutionWidth, resolutionHeight)
    img = np.zeros(shape=(resolutionWidth, resolutionHeight, 3), dtype=np.uint8)
    nt: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)
    setupNetworkTables(conePipeline)
    # loop forever
    while True:
        global currentPipeline
        time, input_img = cvSink.grabFrame(img)
        if time == 0: # There is an error
            outputStream.notifyError(cvSink.getError())
            continue
        index = pipelineNT.get(conePipelineIndex)
        if (index == conePipelineIndex):
            binary_img = executePipeline(input_img, conePipeline)
        elif (index == cubePipelineIndex):
            binary_img = executePipeline(input_img, cubePipeline)
        elif (index == aprilTagsPipelineIndex):
            binary_img = aprilTagPipeline()
        else:
            continue
        # Contours
        contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            mainContour = contours[0]
            for contour in contours:
                if cv2.contourArea(contour) > cv2.contourArea(mainContour):
                    mainContour = contour
            if (cv2.contourArea(mainContour) < 15):
                haveTargetNT.set(False)
                continue
            haveTargetNT.set(True)
            # Bounding Rectangle
            rect = cv2.boundingRect(mainContour)
            x, y, w, h = rect
            center = (int(x + 1/2*w), int(y + 1/2*h)) # Center of bounding rectangle
            crosshair = (int(x + 1/2*w), int(y + h)) # Crosshair on bottom for measurements 
            
            pitch, yaw = pointToPitchAndYaw(crosshair[0], crosshair[1])
            pitchNT.set(pitch)
            yawNT.set(yaw)
            # Convert to color to draw stuff
            binary_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
            binary_img = cv2.drawContours(binary_img, mainContour, -1, color = (255, 0, 0), thickness = 2)
            binary_img = cv2.rectangle(binary_img, (x, y), (x + w, y + h), color = (0, 0, 255), thickness = 2)
            binary_img = cv2.circle(binary_img, center = crosshair, radius = 10, color = (0, 255, 0), thickness = -1)
        else:
            haveTargetNT.set(False)
        outputStream.putFrame(binary_img) # Stream Video
        originalStream.putFrame(input_img) # Stream Video


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
