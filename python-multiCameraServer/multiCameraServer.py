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
from cv2 import Mat
from ntcore import EventFlags, NetworkTable, NetworkTableInstance
from robotpy_apriltag import *

#!
numberOfCameras: Final[int] = 1
team: Final[int] = 2508
server = False
cameraPath: Final[str] = "/dev/video"
cameras = []

class Pipeline:
    
    def __init__(self, hueMin: int, hueMax: int, saturationMin: int, saturationMax: int, valueMin: int, valueMax: int, erodeIterations: int, dilateIterations: int, pipelineIndex: int, minArea: int):
        self.hueMin: Final[int] = hueMin
        self.hueMax: Final[int] = hueMax
        self.saturationMin: Final[int] = saturationMin
        self.saturationMax: Final[int] = saturationMax
        self.valueMin: Final[int] = valueMin
        self.valueMax: Final[int] = valueMax
        self.erodeIterations: Final[int] = erodeIterations
        self.dilateIterations: Final[int] = dilateIterations
        self.pipelineIndex: Final[int] = pipelineIndex
        self.minArea: Final[int] = minArea


# Camera
resolutionWidth: Final[int] = 1280
resolutionHeight: Final[int] = 720
fps: Final[int] = 30
exposure: Final[int] = 40
verticalFOV: Final[int] = math.radians(36.9187406) # rad, Calculated manually but better cause ben
horizontalFOV: Final[int] = math.radians(61.3727249) # rad, Calculated manually but better cause ben
# Load previously saved data
with np.load('cameraCalib.npz') as file:
    mtx, dist = [file[i] for i in ('mtx', 'dist')]

# Processing
conePipelineIndex: Final[int] = 0
cubePipelineIndex: Final[int] = 1
aprilTagsPipelineIndex: Final[int] = 2
conePipeline: Pipeline = Pipeline(8, 40, 120, 255, 160, 255, 1, 1, conePipelineIndex, 15)
cubePipeline: Pipeline = Pipeline(120, 150, 30, 255, 120, 255, 1, 3, cubePipelineIndex, 15)
currentPipeline: int = aprilTagsPipelineIndex

# AprilTags
focalLengthPixels: Final[float] = 1078.466
decimate: Final[int] = 8
aprilTagLength: Final[float] = 0.1524 # Meters
detector: Final[AprilTagDetector] = AprilTagDetector()
detector.addFamily("tag16h5")
detector.setConfig(AprilTagDetector.Config(1, decimate, 0, True, .25, False))
poseEstimator: Final[AprilTagPoseEstimator] = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(aprilTagLength, 1078.466, 1078.466, resolutionWidth/2, resolutionHeight/2))
tagMinArea: Final[int] = 25 # pixels
# Adaptive Thresholding
maxValue: Final[int] = 255
blockSize: Final[int] = 11
constantC: Final[int] = 2

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
minAreaNT: Final = nt.getIntegerTopic("minArea").getEntry(0)
pitchNT: Final = nt.getFloatTopic("Pitch").getEntry(0)
yawNT: Final = nt.getFloatTopic("Yaw").getEntry(0)
haveTargetNT: Final = nt.getBooleanTopic("HaveTarget").getEntry(False)
pipelineNT.set(currentPipeline)

def startCamera():
    camNum: str = str(len(cameras))
    """Start running the camera."""
    print("Starting camera '{}' on {}".format("Camera " + camNum, cameraPath + camNum))
    camera = UsbCamera("Camera " + camNum, cameraPath + camNum)
    CameraServer.startAutomaticCapture(camera=camera)
    camera.setResolution(resolutionWidth, resolutionHeight)
    print("CS: Camera {}: set resolution to {}x{}".format(camNum, resolutionWidth, resolutionHeight))
    camera.setFPS(fps)
    print("CS: Camera {}: set FPS to {}".format(camNum, fps))
    camera.setExposureManual(exposure)
    print("CS: Camera {}: set exposure to {}".format(camNum, exposure))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    return camera

def setupNetworkTables(pipeline: Pipeline):
    hueMinNT.set(pipeline.hueMin)
    hueMaxNT.set(pipeline.hueMax)
    saturationMinNT.set(pipeline.saturationMin)
    saturationMaxNT.set(pipeline.saturationMax)
    valueMinNT.set(pipeline.valueMin)
    valueMaxNT.set(pipeline.valueMax)
    erodeIterationsNT.set(pipeline.erodeIterations)
    dilateIterationsNT.set(pipeline.dilateIterations)
    minAreaNT.set(pipeline.minArea)
    pitchNT.set(0)
    yawNT.set(0)
    haveTargetNT.set(False)

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

def getOrientation(w, h, pipeline: ColorPipeline)

def contourPipelines(binary_img):
    # Contours
    contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        mainContour = contours[0]
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(mainContour):
                mainContour = contour
        if (cv2.contourArea(mainContour) < minAreaNT.get(15)):
            haveTargetNT.set(False)
            return
        haveTargetNT.set(True)
        # Bounding Rectangle
        rect = cv2.boundingRect(mainContour)
        x, y, w, h = rect
        center = (int(x + 1/2*w), int(y + 1/2*h)) # Center of bounding rectangle
        crosshair = (int(x + 1/2*w), int(y + h)) # Crosshair on bottom for measurements 
        
        yaw, pitch = pointToYawAndPitch(crosshair[0], crosshair[1])
        pitchNT.set(pitch)
        yawNT.set(yaw)
        # Convert to color to draw stuff
        binary_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        binary_img = cv2.drawContours(binary_img, mainContour, -1, color = (255, 0, 0), thickness = 2)
        binary_img = cv2.rectangle(binary_img, (x, y), (x + w, y + h), color = (0, 0, 255), thickness = 2)
        binary_img = cv2.circle(binary_img, center = crosshair, radius = 10, color = (0, 255, 0), thickness = -1)
        return binary_img
    else:
        haveTargetNT.set(False)
        return binary_img

def getAreaAprilTag(tag: AprilTagDetection):
    x1: float = abs(tag.getCorner(0).x - tag.getCorner(1).x)
    x2: float = abs(tag.getCorner(0).x - tag.getCorner(2).x)
    y1: float = abs(tag.getCorner(0).y - tag.getCorner(1).y)
    y2: float = abs(tag.getCorner(0).y - tag.getCorner(2).y)
    width: float = max(x1, x2); 
    height: float = max(y1, y2); 
    return width * height

def showDetection(input_img: Mat, result: AprilTagDetection):
    # Crosshair, Pitch and Yaw Stuff
    y1: float = abs(result.getCorner(0).y - result.getCorner(1).y)
    y2: float = abs(result.getCorner(0).y - result.getCorner(2).y)
    height: float = max(y1, y2); # in pixels
    centerX: int = int(result.getCenter().x)
    centerY: int = int(result.getCenter().y)
    crosshair = (int(centerX), int(centerY + (height/2)))
    yaw, pitch = pointToYawAndPitch(crosshair[0], crosshair[1]) 
    pitchNT.set(pitch)
    yawNT.set(yaw)
    estimate = poseEstimator.estimateOrthogonalIteration(result, 100).pose1
    print("X: " + str(estimate.X() * 39.37) + " Y: " + str(estimate.Y() * 39.37) + " Rotation: " + str(math.degrees(estimate.rotation().Z())) + str(result.getId()))
    # Draw Stuff
    # binary_img = drawAxis(binary_img, result.getCorners((0, 0, 0, 0, 0, 0, 0, 0)))
    binary_img = cv2.circle(input_img, center = crosshair, radius = 10, color = (0, 255, 0), thickness = -1)
    binary_img = cv2.rectangle(binary_img, 
        (int(result.getCorner(0).x), int(result.getCorner(0).y)), 
        (int(result.getCorner(2).x), int(result.getCorner(2).y)), 
        color = (0, 0, 255), thickness = 2)
    binary_img = cv2.putText(binary_img, str(result.getId()), org = (centerX, centerY), fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 3, color = (255, 0, 0), thickness = 4)
    return binary_img

def aprilTagPipeline(input_img: Mat):
    # Convert to grayscale
    binary_img: Mat = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY) 
    # binary_img = cv2.blur(binary_img, (5, 5))
    detections = detector.detect(binary_img)
    if (len(detections) > 0):
        result = detections[0]
        for detection in detections:
            colorImg = showDetection(input_img, detection)
        #     if getAreaAprilTag(detection) > getAreaAprilTag(result):
        #         result = detection
        # if (getAreaAprilTag(result) < tagMinArea):
        #     haveTargetNT.set(False)
        #     return
        # haveTargetNT.set(True)
        # binary_img = showDetection(result)
        return colorImg
    haveTargetNT.set(False)
    pitchNT.set(0)
    yawNT.set(0)
    return binary_img

def main(): # Image proccessing user code
    CameraServer.enableLogging()
    cvSink = CameraServer.getVideo()
    outputStream = CameraServer.putVideo("Proccessed Video", resolutionWidth, resolutionHeight)
    originalStream = CameraServer.putVideo("Original Video", resolutionWidth, resolutionHeight)
    img = np.zeros(shape=(resolutionWidth, resolutionHeight, 3), dtype=np.uint8)
    setupNetworkTables(conePipeline)
    # loop forever
    while True:
        global currentPipeline
        time, input_img = cvSink.grabFrame(img)
        if time == 0: # There is an error
            outputStream.notifyError(cvSink.getError())
            continue
        index = pipelineNT.get(aprilTagsPipelineIndex)
        if (index == conePipelineIndex):
            binary_img = executePipeline(input_img, conePipeline)
            binary_img = contourPipelines(binary_img)
        elif (index == cubePipelineIndex):
            binary_img = executePipeline(input_img, cubePipeline)
            binary_img = contourPipelines(binary_img)
        elif (index == aprilTagsPipelineIndex):
            binary_img = aprilTagPipeline(input_img)
        else:
            continue
        outputStream.putFrame(binary_img) # Stream Video
        originalStream.putFrame(input_img) # Stream Video
        
        

if __name__ == "__main__":
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
    for i in range(numberOfCameras):
        cameras.append(startCamera())
    # start opencv code
    main() 
