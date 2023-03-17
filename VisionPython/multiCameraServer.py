#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import math
import platform
import sys
import time
from enum import Enum
from typing import Final

import cv2
import numpy as np
from cscore import CameraServer, UsbCamera, VideoMode, VideoSource
from cv2 import Mat
from ntcore import EventFlags, NetworkTable, NetworkTableInstance
from robotpy_apriltag import (AprilTagDetection, AprilTagDetector,
                              AprilTagPoseEstimate, AprilTagPoseEstimator)
from wpimath.geometry import Transform3d

cv2.setUseOptimized(True)

class CameraConfig: pass

class Camera:
    
    def __init__(self, resolutionWidth: int,  resolutionHeight: int, diagonalFOV: float, fx: float, fy: float, cx: float, cy: float):
        self.resolutionWidth: Final[float] = resolutionWidth
        self.resolutionHeight: Final[float] = resolutionHeight
        self.diagonalFOV: Final[float] = diagonalFOV
        self.fx: Final[float] = fx
        self.fy: Final[float] = fy
        self.cx: Final[float] = cx
        self.cy: Final[float] = cy

def getHorizontalFOVRad(resolutionWidth: int, resolutionHeight: int, diagonalFOVDeg: float) -> float:
    DfRad: float = math.radians(diagonalFOVDeg)
    Da: float = math.sqrt((resolutionWidth * resolutionWidth) + (resolutionHeight * resolutionHeight))
    Hf: float = math.atan(math.tan(DfRad/2) * (resolutionWidth/Da)) * 2
    return Hf
    
def getVerticalFOVRad(resolutionWidth: int, resolutionHeight: int, diagonalFOVDeg: float) -> float:
    DfRad: float = math.radians(diagonalFOVDeg)
    Da: float = math.sqrt((resolutionWidth * resolutionWidth) + (resolutionHeight * resolutionHeight))
    Vf: float = math.atan(math.tan(DfRad/2) * (resolutionHeight/Da)) * 2
    return Vf

configFile = "/boot/frc.json"
team: Final[int] = 2508
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

# AprilTags
detector: Final[AprilTagDetector] = AprilTagDetector()
detector.addFamily("tag16h5")
aprilTagLengthMeters: Final[float] = 0.1524

# NetworkTables
networkTableName: Final[str] = "VisionRPI"
mainTable: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)

lifecam: Camera = Camera(1280, 720, 68.5, 1136.5690362861997, 1141.4736688529676, 641.2131167961169, 367.60929575540644) # calibdb
logitechCam: Camera = Camera(1920, 1080, 170, 1378.7537012386945, 1375.0934365805474, 986.8907369291361, 513.9387512470897) # calibdb

resolutionWidth: int = 640
resolutionHeight: int = 360
verticalFOVRad: float 
horizontalFOVRad: float 
focalLengthPixels: float
fps: Final[int] = 30
exposure: Final[int] = 35


def onRPI() -> bool:
    return platform.uname().system == "Linux"

def parseError(str) -> None:
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config) -> bool:
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

def readSwitchedCameraConfig(config) -> bool:
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

def readConfig() -> bool:
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

def startCamera(config) -> UsbCamera:
    global resolutionWidth, resolutionHeight
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)
    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    
    resolutionWidth = config.config["width"]
    resolutionHeight = config.config["height"]

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

def startCameraDesktop() -> None:
    camNum: str = "1"
    print("Starting camera {}".format("USB Camera " + camNum))
    camera = CameraServer.startAutomaticCapture(int(camNum))
    # print("CS: USB Camera {}: Setting Video Mode to PixelFormat {}, Width {}, Height {} and FPS {}".format(camNum, VideoMode.PixelFormat.kMJPEG, resolutionWidth, resolutionHeight, fps))
    print("CS: USB Camera {}: Setting Video Mode to PixelFormat {}, Width {}, Height {} and FPS {}".format(camNum, VideoMode.PixelFormat.kYUYV, resolutionWidth, resolutionHeight, fps))
    print("CS: USB Camera {}: Set Video Mode Successfully ? ".format(camNum) + str(camera.setVideoMode(VideoMode.PixelFormat.kYUYV, resolutionWidth, resolutionHeight, fps)))
    camera.setExposureManual(exposure)
    print("CS: USB Camera {}: Setting exposure to {}".format(camNum, exposure))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    print("CS: USB Camera {}: Setting connection strategy to {}".format(camNum, VideoSource.ConnectionStrategy.kConnectionKeepOpen))

class Pipeline:
    
    class Type(Enum):
        COLOR = 0
        APRILTAG = 1
    
    def __init__(self, index: int, type: Type, subTable: str) -> None:
        self.type: Final[Pipeline.Type] = type
        self.subTable: Final[NetworkTable] = mainTable.getSubTable(subTable)
        self.__setupTableEntries(index)
        
    def __setupTableEntries(self, index):
        self.pipelineIndex = self.subTable.getEntry("Pipeline")
        self.hasTarget = self.subTable.getEntry("Has Target")
        self.hasTarget.setBoolean(False)
        self.pipelineIndex.setInteger(index)
        
class AprilTagPipeline(Pipeline):
    
    def __init__(self, pipelineIndex: int, name: str, detectorConfig: AprilTagDetector.Config, quadParameters: AprilTagDetector.QuadThresholdParameters, poseIterations: int, minArea: float, minDecisionMargin: float) -> None:
        super().__init__(pipelineIndex, super().Type.APRILTAG, name)
        self.detectorConfig: Final[AprilTagDetector.Config] = detectorConfig
        self.detectorQuadParameters: Final[AprilTagDetector.QuadThresholdParameters] = quadParameters
        self.__setupTableEntries(poseIterations, minArea, minDecisionMargin)
    
    def __setupTableEntries(self, poseIterations, minArea, minDecisionMargin):
        self.poseIterations = self.subTable.getEntry("DEBUG: Pose Iterations")
        self.minArea = self.subTable.getEntry("DEBUG: Min Area")
        self.minDecisionMargin = self.subTable.getEntry("DEBUG: Min Decision Margin")
        self.X = self.subTable.getEntry("X") 
        self.Y = self.subTable.getEntry("Y")
        self.Z = self.subTable.getEntry("Z")
        self.Yaw = self.subTable.getEntry("Yaw")
        self.poseIterations.setInteger(poseIterations)
        self.minArea.setDouble(minArea)
        self.minDecisionMargin.setDouble(minDecisionMargin)
        self.X.setDouble(0)
        self.Y.setDouble(0)
        self.Z.setDouble(0)
        self.Yaw.setDouble(0)

class ColorPipeline(Pipeline):
    
    def __init__(self, pipelineIndex: int, name: str, hueMin: int, hueMax: int, saturationMin: int, saturationMax: int, valueMin: int, valueMax: int, erodeIterations: int, dilateIterations: int, minArea: int) -> None:
        super().__init__(pipelineIndex, super().Type.COLOR, name)
        self.__setupTableEntries(minArea, hueMin, hueMax, saturationMin, saturationMax, valueMin, valueMax, erodeIterations, dilateIterations)
        
    def __setupTableEntries(self, minArea, hueMin, hueMax, saturationMin, saturationMax, valueMin, valueMax, erodeIterations, dilateIterations):
        self.hueMin = self.subTable.getEntry("DEBUG: Hue Min")
        self.hueMax = self.subTable.getEntry("DEBUG: Hue Max")
        self.saturationMin = self.subTable.getEntry("DEBUG: Saturation Min")
        self.saturationMax = self.subTable.getEntry("DEBUG: Saturation Max")
        self.valueMin = self.subTable.getEntry("DEBUG: Value Min")
        self.valueMax = self.subTable.getEntry("DEBUG: Value Max")
        self.erodeIterations = self.subTable.getEntry("DEBUG: Erode Iterations")
        self.dilateIterations = self.subTable.getEntry("DEBUG: Dilate Iterations")
        self.minArea = self.subTable.getEntry("Min Area")
        self.pitch = self.subTable.getEntry("Pitch") 
        self.yaw = self.subTable.getEntry("Yaw")
        self.orientation = self.subTable.getEntry("Orientation")
        self.hueMin.setInteger(hueMin)
        self.hueMax.setInteger(hueMax)
        self.saturationMin.setInteger(saturationMin)
        self.saturationMax.setInteger(saturationMax)
        self.valueMin.setInteger(valueMin)
        self.valueMax.setInteger(valueMax)
        self.erodeIterations.setInteger(erodeIterations)
        self.dilateIterations.setInteger(dilateIterations)
        self.minArea.setDouble(minArea)
        self.pitch.setDouble(0)
        self.yaw.setDouble(0)
        self.orientation.setInteger(0)

# Processing
conePipeline: ColorPipeline = ColorPipeline(0, "Cone", 15, 40, 150, 255, 180, 255, 1, 1, 150)
cubePipeline: ColorPipeline = ColorPipeline(0, "Cube", 120, 150, 30, 255, 80, 255, 1, 4, 150)
config: AprilTagDetector.Config = AprilTagDetector.Config()
config.quadDecimate = 1
config.decodeSharpening = 0.25
config.quadSigma = 0
config.refineEdges = True
config.numThreads = 2
config.debug = False
quadThreshold: AprilTagDetector.QuadThresholdParameters = AprilTagDetector.QuadThresholdParameters()
tagPipeline: AprilTagPipeline = AprilTagPipeline(2, "AprilTag", config, quadThreshold, 100, 25, 35)
pipelines: tuple[Pipeline, ...] = (conePipeline, cubePipeline, tagPipeline)
poseEstimator: AprilTagPoseEstimator # defined in main() so the parameters are correct

def getAreaAprilTag(tag: AprilTagDetection) -> float:
    x1: float = abs(tag.getCorner(0).x - tag.getCorner(1).x)
    x2: float = abs(tag.getCorner(0).x - tag.getCorner(2).x)
    y1: float = abs(tag.getCorner(0).y - tag.getCorner(1).y)
    y2: float = abs(tag.getCorner(0).y - tag.getCorner(2).y)
    width: float = max(x1, x2); 
    height: float = max(y1, y2); 
    return width * height

def getTagData(pipeline: AprilTagPipeline, result: AprilTagDetection) -> None:
    bestResult = Transform3d()
    # bestResult = poseEstimator.estimateHomography(result)
    estimate: AprilTagPoseEstimate = poseEstimator.estimateOrthogonalIteration(result, pipeline.poseIterations.getInteger(0))
    if (estimate.error1 <= estimate.error2):
        bestResult = estimate.pose1
    else:
        bestResult = estimate.pose2
    pipeline.X.setDouble(bestResult.X())
    pipeline.Y.setDouble(bestResult.Y())
    pipeline.Z.setDouble(bestResult.Z())
    pipeline.Yaw.setDouble(math.degrees(bestResult.rotation().Y()))

def drawDetection(drawnImg: Mat, result: AprilTagDetection) -> Mat:
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
    drawnImg = cv2.putText(drawnImg, str(result.getId()), org = (centerX+11, centerY), fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 2, color = (255, 0, 0), thickness = 3)
    return drawnImg

def aprilTagPipeline(input_img: Mat, drawnImg: Mat, pipeline: AprilTagPipeline) -> Mat:
    # Convert to grayscale
    binary_img: Mat = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY) 
    detector.setConfig(pipeline.detectorConfig)
    detector.setQuadThresholdParameters(pipeline.detectorQuadParameters)
    detections = detector.detect(binary_img)
    if (len(detections) > 0):
        result = detections[0]
        for detection in detections:
            if (detection.getDecisionMargin() > result.getDecisionMargin() and getAreaAprilTag(detection) > getAreaAprilTag(result)):
                result = detection
        if (getAreaAprilTag(result) < pipeline.minArea.getDouble(0) or result.getDecisionMargin() < pipeline.minDecisionMargin.getDouble(0)):
            pipeline.hasTarget.setBoolean(False)
            pipeline.X.setDouble(0)
            pipeline.Y.setDouble(0)
            pipeline.Z.setDouble(0)
            pipeline.Yaw.setDouble(0)
            return drawnImg
        drawDetection(drawnImg, result)
        getTagData(pipeline, result)
        pipeline.hasTarget.setBoolean(True)
        return drawnImg
    pipeline.hasTarget.setBoolean(False)
    pipeline.X.setDouble(0)
    pipeline.Y.setDouble(0)
    pipeline.Z.setDouble(0)
    pipeline.Yaw.setDouble(0)
    return drawnImg

def pointToYawAndPitch(px: int, py: int) -> tuple[float, float]: # Converts a point in pixel system to a pitch and a yaw and returns that.
    # print("{}, {}, {}, {}".format(resolutionWidth, resolutionHeight, math.degrees(horizontalFOVRad), math.degrees(verticalFOVRad)))
    nx: float = (2/resolutionWidth) * (px - ((resolutionWidth/2) - 0.5))
    ny: float = (2/resolutionHeight) * (((resolutionHeight/2) - 0.5) - py)
    vpw: float = 2.0*math.tan(horizontalFOVRad/2)
    vph: float = 2.0*math.tan(verticalFOVRad/2)
    x: float = vpw/2 * nx
    y: float = vph/2 * ny
    ax: float = math.atan(x/1)
    ay: float = math.atan(y/1)
    yaw: float = math.degrees(ax)
    pitch: float = math.degrees(ay)
    return (yaw, pitch)

def getOrientation(width: float, height: float, pipeline: ColorPipeline) -> None:
    if (width > height) or (width == height):
        pipeline.orientation.setInteger(0) #0 means landscape
    else:
        pipeline.orientation.setInteger(1) #1 means portrait

def colorPipeline(img: Mat, drawnImg: Mat, pipeline: ColorPipeline) -> Mat:
    # Color Converting
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Color Thresholding
    binaryImg = cv2.inRange(hsvImg, (
        pipeline.hueMin.getInteger(0),
        pipeline.saturationMin.getInteger(0), 
        pipeline.valueMin.getInteger(0)
    ), (
        pipeline.hueMax.getInteger(0), 
        pipeline.saturationMax.getInteger(0), 
        pipeline.valueMax.getInteger(0)
    ))
    # Eroding
    kernel = np.ones((3, 3), np.uint8)
    binaryImg = cv2.erode(binaryImg, kernel, iterations = int(pipeline.erodeIterations.getInteger(0)))
    # Dilating
    kernel = np.ones((3, 3), np.uint8)
    binaryImg = cv2.dilate(binaryImg, kernel, iterations = int(pipeline.dilateIterations.getInteger(0)))
    return proccessContours(binaryImg, drawnImg, pipeline)

def proccessContours(binaryImg: Mat, drawnImg: Mat, pipeline: ColorPipeline) -> Mat:
    # Contours
    contours, hierarchy = cv2.findContours(binaryImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        mainContour = contours[0]
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(mainContour):
                mainContour = contour
        if (cv2.contourArea(mainContour) < pipeline.minArea.getDouble(0)):
            pipeline.hasTarget.setBoolean(False)
            return drawnImg
        pipeline.hasTarget.setBoolean(True)
        # Bounding Rectangle
        rect = cv2.boundingRect(mainContour)
        x, y, w, h = rect
        getOrientation(w, h, pipeline)
        crosshair = (int(x + 1/2*w), int(y + h)) # Crosshair on bottom for measurements 
        yaw, pitch = pointToYawAndPitch(crosshair[0], crosshair[1])
        pipeline.pitch.setDouble(pitch)
        pipeline.yaw.setDouble(yaw)
        # Draw Stuff
        drawnImg = cv2.drawContours(drawnImg, mainContour, -1, color = (255, 0, 0), thickness = 2)
        drawnImg = cv2.rectangle(drawnImg, (x, y), (x + w, y + h), color = (0, 0, 255), thickness = 2)
        drawnImg = cv2.circle(drawnImg, center = crosshair, radius = 5, color = (0, 255, 0), thickness = -1)
        return drawnImg
    else:
        pipeline.hasTarget.setBoolean(False)
        return drawnImg
       
def setupCameraConstants() -> None:
    global horizontalFOVRad, verticalFOVRad  
    horizontalFOVRad = getHorizontalFOVRad(resolutionWidth, resolutionHeight, lifecam.diagonalFOV)
    verticalFOVRad = getVerticalFOVRad(resolutionWidth, resolutionHeight, lifecam.diagonalFOV)
    
def main() -> None: # Image proccessing user code
    CameraServer.enableLogging()
    if onRPI():
        cvSinkHigh = CameraServer.getVideo("Camera Color") 
        cvSinkLow = CameraServer.getVideo("Camera Tag") 
    else:
        cvSinkHigh = CameraServer.getVideo()
        cvSinkLow = CameraServer.getVideo()
    proccessedStream = CameraServer.putVideo("Proccessed Video", resolutionWidth, resolutionHeight)
    originalStream = CameraServer.putVideo("Original Video", resolutionWidth, resolutionHeight)
    mat = np.zeros(shape=(resolutionWidth, resolutionHeight, 3), dtype=np.uint8)
    mainTable.getEntry("Current Pipeline").setInteger(tagPipeline.pipelineIndex.getInteger(0))
    global poseEstimator
    scalingFactor: float = (resolutionWidth / lifecam.resolutionWidth)
    poseEstimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(
        aprilTagLengthMeters, lifecam.fx * scalingFactor, lifecam.fy * scalingFactor, lifecam.cx * scalingFactor, lifecam.cy * scalingFactor
    ))
    config = poseEstimator.getConfig()
    print("Pose Estimator Config -> tagSize: {} meters, fx: {} pixels, fy: {} pixels, cx: {} pixels, cy: {} pixels".format(
        config.tagSize, config.fx, config.fy, config.cx, config.cy
    ))
    # loop forever
    while True:
        ts = time.time()
        index = mainTable.getEntry("Current Pipeline").getInteger(0)
        cvSink = cvSinkLow
        for pipeline in pipelines:
            if (pipeline.pipelineIndex.getInteger(0) == index):
                if (pipeline.type == Pipeline.Type.COLOR):
                    cvSink = cvSinkHigh
                elif (pipeline.type == Pipeline.Type.APRILTAG):
                    cvSink = cvSinkLow
        error, inputImg = cvSink.grabFrame(mat)
        inputImg: Mat
        # # * Undistort the camera's image
        # if (mtx is not None and dist is not None):
        #     # inputImg = cv2.undistort(inputImg, mtx, dist)
        #     pass
        if error == 0: # There is an error
            print("CVSINK ERROR: " + cvSink.getError())
            continue
        drawnImg = inputImg.copy()
        for pipeline in pipelines:
            if (pipeline.pipelineIndex.getInteger(0) == index):
                if (pipeline.type == Pipeline.Type.COLOR):
                    drawnImg = colorPipeline(inputImg, drawnImg, pipeline)
                elif (pipeline.type == Pipeline.Type.APRILTAG):
                    drawnImg = aprilTagPipeline(inputImg, drawnImg, pipeline)
            else:
                pipeline.hasTarget.setBoolean(False)
        drawnImg = cv2.circle(drawnImg, (int(resolutionWidth/2), int(resolutionHeight/2)), 5, (255, 255, 255), -1)
        try:
            proccessedStream.putFrame(drawnImg) # Stream Video
            originalStream.putFrame(inputImg) # Stream Video
        except:
            continue
        # print(time.time() - ts)
        
        

if __name__ == "__main__":
    if (onRPI()):
        if len(sys.argv) >= 2:
            configFile = sys.argv[1]

        # read configuration
        if not readConfig():
            sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server or not onRPI():
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()
    if (onRPI()):
        print("Running On RaspberryPi")
        # start cameras
        for config in cameraConfigs:
            startCamera(config)

        # start switched cameras
        for config in switchedCameraConfigs:
            startSwitchedCamera(config)
    else:
        print("Running On Desktop")
        startCameraDesktop()
    # start opencv
    main()
