#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import math
import platform
import sys
from enum import Enum
from typing import Final

import cv2
import numpy as np
from cscore import CameraServer, MjpegServer, UsbCamera, VideoSource
from cv2 import Mat
from ntcore import EventFlags, NetworkTable, NetworkTableInstance
from robotpy_apriltag import (AprilTag, AprilTagDetection, AprilTagDetector,
                              AprilTagPoseEstimate, AprilTagPoseEstimator)


class CameraConfig: pass

configFile = "/boot/frc.json"
team: Final[int] = 2508
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

# AprilTags
detector: Final[AprilTagDetector] = AprilTagDetector()
detector.addFamily("tag16h5")
aprilTagLength: Final[float] = 0.1524 # Meters

# NetworkTables
networkTableName: Final[str] = "VisionRPI"
mainTable: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)

def onRPI() -> bool:
    return platform.uname().system == "Linux"

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

def startCameraDesktop():
    camNum: str = str(len(cameras))
    """Start running the camera."""
    print("Starting camera '{}' on {}".format("Camera " + camNum, camNum))
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(resolutionWidth, resolutionHeight)
    print("CS: Camera {}: set resolution to {}x{}".format(camNum, resolutionWidth, resolutionHeight))
    camera.setFPS(fps)
    print("CS: Camera {}: set FPS to {}".format(camNum, fps))
    camera.setExposureManual(exposure)
    print("CS: Camera {}: set exposure to {}".format(camNum, exposure))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    return camera

class Pipeline:
    
    class Type(Enum):
        COLOR = 0
        APRILTAG = 1
    
    def __init__(self, index: int, type: Type, subTable: str) -> None:
        self.pipelineIndex: Final[int] = index
        self.type: Final[self.Type] = type
        self.subTable: Final[NetworkTable] = mainTable.getSubTable(subTable)
        self.__setupTableEntries()
        
    def __setupTableEntries(self):
        self.HasTarget = self.subTable.getEntry("HasTarget")
        self.HasTarget.setBoolean(False)
        
class AprilTagPipeline(Pipeline):
    
    def __init__(self, index: int, name: str, numThreads: int, quadDecimate: float, quadSigma: float, refineEdges: bool, decodeSharpening: float, debug: bool) -> None:
        global detector
        super().__init__(index, super().Type.APRILTAG, name)
        self.detectorConfig: Final[AprilTagDetector.Config] = AprilTagDetector.Config()
        self.detectorConfig.numThreads = numThreads
        self.detectorConfig.quadDecimate = quadDecimate
        self.detectorConfig.quadSigma = quadSigma
        self.detectorConfig.refineEdges = refineEdges
        self.detectorConfig.decodeSharpening = decodeSharpening
        self.detectorConfig.debug = debug
        self.__setupTableEntries()
    
    def __setupTableEntries(self):
        self.X = self.subTable.getEntry("X") 
        self.Y = self.subTable.getEntry("Y")
        self.Z = self.subTable.getEntry("Z")
        self.Yaw = self.subTable.getEntry("Yaw")
        self.X.setDouble(0)
        self.Y.setDouble(0)
        self.Z.setDouble(0)
        self.Yaw.setDouble(0)

class ColorPipeline(Pipeline):
    
    def __init__(self, index: int, name: str, hueMin: int, hueMax: int, saturationMin: int, saturationMax: int, valueMin: int, valueMax: int, erodeIterations: int, dilateIterations: int, minArea: int) -> None:
        super().__init__(index, super().Type.COLOR, name)
        self.hueMin: Final[int] = hueMin
        self.hueMax: Final[int] = hueMax
        self.saturationMin: Final[int] = saturationMin
        self.saturationMax: Final[int] = saturationMax
        self.valueMin: Final[int] = valueMin
        self.valueMax: Final[int] = valueMax
        self.erodeIterations: Final[int] = erodeIterations
        self.dilateIterations: Final[int] = dilateIterations
        self.minArea: Final[int] = minArea

# Camera
resolutionWidth: Final[int] = 1280
resolutionHeight: Final[int] = 720
fps: Final[int] = 30
exposure: Final[int] = 40
verticalFOV: Final[int] = math.radians(36.9187406) # rad, Calculated manually 
horizontalFOV: Final[int] = math.radians(61.3727249) # rad, Calculated manually
focalLengthPixels: Final[float] = 1078.466
# Load previously saved data
with np.load('./cameraCalib.npz') as file:
    mtx, dist = [file[i] for i in ('mtx', 'dist')]

# Processing
conePipeline: ColorPipeline = ColorPipeline(0, "Cone", 8, 40, 120, 255, 160, 255, 1, 1, 15)
cubePipeline: ColorPipeline = ColorPipeline(1, "Cube", 120, 150, 30, 255, 120, 255, 1, 3, 15)
tagPipeline: AprilTagPipeline = AprilTagPipeline(2, "AprilTag", 1, 2, 0, True, 0.25, False)
pipelines: list[Pipeline] = [conePipeline, cubePipeline, tagPipeline] 

poseEstimator: Final[AprilTagPoseEstimator] = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(aprilTagLength, 1078.466, 1078.466, resolutionWidth/2, resolutionHeight/2))
tagMinArea: Final[int] = 25 # pixels
tagMinDecisionMargin: Final[int] = 20
# Adaptive Thresholding
maxValue: Final[int] = 255
blockSize: Final[int] = 11
constantC: Final[int] = 2

def getAreaAprilTag(tag: AprilTagDetection):
    x1: float = abs(tag.getCorner(0).x - tag.getCorner(1).x)
    x2: float = abs(tag.getCorner(0).x - tag.getCorner(2).x)
    y1: float = abs(tag.getCorner(0).y - tag.getCorner(1).y)
    y2: float = abs(tag.getCorner(0).y - tag.getCorner(2).y)
    width: float = max(x1, x2); 
    height: float = max(y1, y2); 
    return width * height

def getTagData(pipeline: AprilTagPipeline, result: AprilTagDetection):
    metersToInches: Final[float] = 39.37
    estimate = poseEstimator.estimateOrthogonalIteration(result, 100).pose1
    pipeline.X.setDouble(estimate.X() * metersToInches)
    pipeline.Y.setDouble(estimate.Y() * metersToInches)
    pipeline.Z.setDouble(estimate.Z() * metersToInches)
    pipeline.Yaw.setDouble(math.degrees(estimate.rotation().Z()))

def drawDetection(drawnImg: Mat, result: AprilTagDetection):
    # Crosshair
    y1: float = abs(result.getCorner(0).y - result.getCorner(1).y)
    y2: float = abs(result.getCorner(0).y - result.getCorner(2).y)
    height: float = max(y1, y2); # in pixels
    centerX: int = int(result.getCenter().x)
    centerY: int = int(result.getCenter().y)
    crosshair = int(centerX), int(centerY + (height/2))
    # Draw Stuff
    drawnImg = cv2.circle(drawnImg, center = crosshair, radius = 10, color = (0, 255, 0), thickness = -1)
    drawnImg = cv2.rectangle(drawnImg, 
        (int(result.getCorner(0).x), int(result.getCorner(0).y)), 
        (int(result.getCorner(2).x), int(result.getCorner(2).y)), 
        color = (0, 0, 255), thickness = 2)
    drawnImg = cv2.putText(drawnImg, str(result.getId()), org = (centerX+11, centerY), fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1, color = (255, 0, 0), thickness = 3)
    return drawnImg

def aprilTagPipeline(input_img: Mat, drawnImg: Mat, pipeline: AprilTagPipeline):
    # Convert to grayscale
    binary_img: Mat = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY) 
    detector.setConfig(pipeline.detectorConfig)
    detections = detector.detect(binary_img)
    if (len(detections) > 0):
        result = detections[0]
        for detection in detections:
            if (detection.getDecisionMargin() > result.getDecisionMargin() and getAreaAprilTag(detection) > getAreaAprilTag(result)):
                result = detection
        if (getAreaAprilTag(result) < tagMinArea or result.getDecisionMargin() < tagMinDecisionMargin):
            pipeline.HasTarget.setBoolean(False)
            pipeline.X.setDouble(0)
            pipeline.Y.setDouble(0)
            pipeline.Z.setDouble(0)
            pipeline.Yaw.setDouble(0)
            return drawnImg
        drawDetection(drawnImg, result)
        getTagData(pipeline, result)
        pipeline.HasTarget.setBoolean(True)
        return drawnImg
    pipeline.HasTarget.setBoolean(False)
    pipeline.X.setDouble(0)
    pipeline.Y.setDouble(0)
    pipeline.Z.setDouble(0)
    pipeline.Yaw.setDouble(0)
    return drawnImg

def pointToYawAndPitch(px: int, py: int): # Converts a point in pixel system to a pitch and a yaw and returns that.
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
    return (yaw, pitch)

def colorPipeline(img: Mat, drawnImg: Mat, pipeline: ColorPipeline):
    # Color Converting
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Color Thresholding
    binaryImg = cv2.inRange(hsvImg, (
        pipeline.hueMin,
        pipeline.saturationMin, 
        pipeline.valueMin
    ), (
        pipeline.hueMax, 
        pipeline.saturationMax, 
        pipeline.valueMax
    ))
    # Eroding
    kernel = np.ones((3, 3), np.uint8)
    binaryImg = cv2.erode(binaryImg, kernel, iterations = int(pipeline.erodeIterations))
    # Dilating
    kernel = np.ones((3, 3), np.uint8)
    binaryImg = cv2.dilate(binaryImg, kernel, iterations = int(pipeline.dilateIterations))
    return proccessContours(binaryImg, drawnImg, pipeline)

def proccessContours(binaryImg: Mat, drawnImg: Mat, pipeline: ColorPipeline) -> Mat:
    # Contours
    contours, hierarchy = cv2.findContours(binaryImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        mainContour = contours[0]
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(mainContour):
                mainContour = contour
        if (cv2.contourArea(mainContour) < pipeline.minArea):
            pipeline.HasTarget.setBoolean(False)
            return
        pipeline.HasTarget.setBoolean(True)
        # Bounding Rectangle
        rect = cv2.boundingRect(mainContour)
        x, y, w, h = rect
        crosshair = (int(x + 1/2*w), int(y + h)) # Crosshair on bottom for measurements 
        
        yaw, pitch = pointToYawAndPitch(crosshair[0], crosshair[1])
        # table.getEntry(pitchNT).setFloat(pitch)
        # table.getEntry(yawNT).setFloat(yaw)
        # Draw Stuff
        drawnImg = cv2.drawContours(drawnImg, mainContour, -1, color = (255, 0, 0), thickness = 2)
        drawnImg = cv2.rectangle(drawnImg, (x, y), (x + w, y + h), color = (0, 0, 255), thickness = 2)
        drawnImg = cv2.circle(drawnImg, center = crosshair, radius = 10, color = (0, 255, 0), thickness = -1)
        return drawnImg
    else:
        pipeline.HasTarget.setBoolean(False)
        return drawnImg
        
def main(): # Image proccessing user code
    CameraServer.enableLogging()
    cvSink = CameraServer.getVideo()
    proccessedStream = CameraServer.putVideo("Proccessed Video", resolutionWidth, resolutionHeight)
    originalStream = CameraServer.putVideo("Original Video", resolutionWidth, resolutionHeight)
    mat = np.zeros(shape=(resolutionWidth, resolutionHeight, 3), dtype=np.uint8)
    mainTable.getEntry("Current Pipeline").setInteger(tagPipeline.pipelineIndex)
    # loop forever
    while True:
        time, inputImg = cvSink.grabFrame(mat)
        inputImg = cv2.undistort(inputImg, mtx, dist)
        if time == 0: # There is an error
            proccessedStream.notifyError(cvSink.getError())
            continue
        drawnImg = inputImg.copy()
        index = mainTable.getEntry("Current Pipeline").getInteger(0)
        for pipeline in pipelines:
            if (pipeline.pipelineIndex == index):
                if (pipeline.type == Pipeline.Type.COLOR):
                    drawnImg = colorPipeline(inputImg.copy(), drawnImg, pipeline)
                elif (pipeline.type == Pipeline.Type.APRILTAG):
                    drawnImg = aprilTagPipeline(inputImg.copy(), drawnImg, pipeline)
        proccessedStream.putFrame(drawnImg) # Stream Video
        originalStream.putFrame(inputImg) # Stream Video
        
        

if __name__ == "__main__":
    if (onRPI()):
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

    if (onRPI()):
        print("On RaspberryPi")
        # start cameras
        for config in cameraConfigs:
            startCamera(config)

        # start switched cameras
        for config in switchedCameraConfigs:
            startSwitchedCamera(config)
    else:
        print("On Desktop")
        startCameraDesktop()
    # start opencv
    main()
