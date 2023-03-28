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
np.set_printoptions(suppress=True)

class CameraConfig: pass

class Camera:
	
	def __init__(self, resolutionWidth: int,  resolutionHeight: int, diagonalFOV: float, fx: float, fy: float, cx: float, cy: float, distCoeffs):
		self.resolutionWidth: Final[float] = resolutionWidth
		self.resolutionHeight: Final[float] = resolutionHeight
		self.diagonalFOV: Final[float] = diagonalFOV
		self.fx: Final[float] = fx
		self.fy: Final[float] = fy
		self.cx: Final[float] = cx
		self.cy: Final[float] = cy
		self.distCoeffs: Final = distCoeffs

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
maxTagID: Final[int] = 8

# NetworkTables
networkTableName: Final[str] = "VisionRPI"
mainTable: NetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName)

lifecam: Camera = Camera(1280, 720, 68.5, 1136.5690362861997, 1141.4736688529676, 641.2131167961169, 367.60929575540644, np.array([
	0.12308428809814226,
	-0.8911552442414836,
	0.0036990419967787612,
	0.0009950580085113828,
	1.8819261759956036
]).astype(np.float32))
wideangleCam: Camera = Camera(1920, 1080, -1, 1068.25296, 1067.62212, 963.436053, 588.321748, np.array([
	-0.40307692, 
	0.18071752, 
	0.00181403, 
	-0.00061986, 
	-0.04021117
]).astype(np.float32))
logitechCam: Camera = Camera(1920, 1080, 170, 1378.7537012386945, 1375.0934365805474, 986.8907369291361, 513.9387512470897, np.zeros(5))

currentCameraType = wideangleCam
camera = None

resolutionWidth: int = 1280
resolutionHeight: int = 720
camNum: Final[int] = 0
verticalFOVRad: float 
horizontalFOVRad: float 
focalLengthPixels: float
cameraMatrix: np.ndarray

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

def startCamera(config):
	global camera, resolutionWidth, resolutionHeight
 
	camera = cv2.VideoCapture(camNum, cv2.CAP_V4L2)
 
	print(config.config)
	
	resolutionWidth = config.config["width"]
	resolutionHeight = config.config["height"]

	camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolutionWidth)
	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolutionHeight)
 
	camera.set(cv2.CAP_PROP_EXPOSURE, config.config["exposure"]) # range of 0-63
 
	while not camera.isOpened():
		print("Attempting to access camera stream")
		time.sleep(1)
	print("Camera stream opened")

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
	print("Starting camera {}".format("USB Camera " + str(camNum)))
	global camera
	camera = cv2.VideoCapture(camNum, cv2.CAP_DSHOW)
	camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolutionWidth)
	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolutionHeight)
	camera.set(cv2.CAP_PROP_EXPOSURE, -7)
	while not camera.isOpened():
		print("Attempting to access camera stream")
		time.sleep(1)
	print("Camera stream opened")

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
		self.tagID = self.subTable.getEntry("id")
		self.tX = self.subTable.getEntry("tX") 
		self.tY = self.subTable.getEntry("tY")
		self.tZ = self.subTable.getEntry("tZ")
		self.rX = self.subTable.getEntry("rX")
		self.rY = self.subTable.getEntry("rY")
		self.rZ = self.subTable.getEntry("rZ")
		self.poseIterations.setInteger(poseIterations)
		self.minArea.setDouble(minArea)
		self.minDecisionMargin.setDouble(minDecisionMargin)
		self.tagID.setInteger(0)
		self.tX.setDouble(0)
		self.tY.setDouble(0)
		self.tZ.setDouble(0)
		self.rX.setDouble(0)
		self.rY.setDouble(0)
		self.rZ.setDouble(0)

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
		self.pixelX = self.subTable.getEntry("Pixel X")
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
		self.pixelX.setInteger(0)

# Processing
conePipeline: ColorPipeline = ColorPipeline(0, "Cone", 15, 40, 150, 255, 180, 255, 1, 1, 150)
cubePipeline: ColorPipeline = ColorPipeline(1, "Cube", 120, 150, 30, 255, 80, 255, 1, 4, 150)
config: AprilTagDetector.Config = AprilTagDetector.Config()
config.quadDecimate = 1
config.decodeSharpening = 0.25
config.quadSigma = 0
config.refineEdges = True
config.numThreads = 2
config.debug = False
quadThreshold: AprilTagDetector.QuadThresholdParameters = AprilTagDetector.QuadThresholdParameters()
tagPipeline: AprilTagPipeline = AprilTagPipeline(2, "AprilTag", config, quadThreshold, 100, 100, 35)
pipelines: tuple[Pipeline, ...] = (conePipeline, cubePipeline, tagPipeline)

def getAreaAprilTag(tag: AprilTagDetection) -> float:
	x1: float = abs(tag.getCorner(0).x - tag.getCorner(1).x)
	x2: float = abs(tag.getCorner(0).x - tag.getCorner(2).x)
	y1: float = abs(tag.getCorner(0).y - tag.getCorner(1).y)
	y2: float = abs(tag.getCorner(0).y - tag.getCorner(2).y)
	width: float = max(x1, x2); 
	height: float = max(y1, y2); 
	return width * height

def getTagDataPNPGeneric(pipeline: AprilTagPipeline, result: AprilTagDetection) -> None:
	length = aprilTagLengthMeters
	objectPoints = np.array([
		[-length / 2, length / 2, 0],
		[length / 2, length / 2, 0],
		[length / 2, -length / 2, 0],
		[-length / 2, -length / 2, 0]
	])
	pts = np.asarray(result.getCorners(np.zeros(8)))
	pts = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)
	pts = pts.reshape((-1, 2))
	pts = pts[::-1]
	imagePoints = pts.astype(np.float32)
	dist = currentCameraType.distCoeffs
	if currentCameraType is wideangleCam:
		dist = np.zeros(5)
	ret, rVecs, tVecs, rerr = cv2.solvePnPGeneric(objectPoints, imagePoints, cameraMatrix = cameraMatrix, distCoeffs = dist, flags = cv2.SOLVEPNP_IPPE_SQUARE) 
	tvec = [0, 0, 0]
	rvec = [0, 0, 0]
	foundMatch = False
	for i, tvec in enumerate(tVecs):
		if(tvec[2] < 0): # z less than 0, skip it
			continue
		rvec = rVecs[i]
		tvec = tVecs[i]
		foundMatch = True
		break
	
	if (foundMatch == False):
		pipeline.hasTarget.setBoolean(False)
		return

	pipeline.tX.setDouble(tvec[0])
	pipeline.tY.setDouble(tvec[1])
	pipeline.tZ.setDouble(tvec[2])
	pipeline.rX.setDouble(rvec[0])
	pipeline.rY.setDouble(rvec[1])
	pipeline.rZ.setDouble(rvec[2])
	pipeline.hasTarget.setBoolean(True)
	
def drawDetection(drawnImg: Mat, result: AprilTagDetection) -> Mat:
	# Crosshair
	y1: float = abs(result.getCorner(0).y - result.getCorner(1).y)
	y2: float = abs(result.getCorner(0).y - result.getCorner(2).y)
	height: float = max(y1, y2); # in pixels
	centerX: int = int(result.getCenter().x)
	centerY: int = int(result.getCenter().y)
	crosshair = int(centerX), int(centerY + (height/2))
	# Draw Stuff
	drawnImg = cv2.circle(drawnImg, center = crosshair, radius = 5, color = (0, 255, 0), thickness = -1)
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
			if (getAreaAprilTag(detection) > getAreaAprilTag(result) and result.getDecisionMargin() < pipeline.minDecisionMargin.getDouble(0) and result.getId() <= maxTagID):
				result = detection
		if (getAreaAprilTag(result) < pipeline.minArea.getDouble(0) or result.getDecisionMargin() < pipeline.minDecisionMargin.getDouble(0) or result.getId() > maxTagID):
			pipeline.hasTarget.setBoolean(False)
			return drawnImg
		drawDetection(drawnImg, result)
		getTagDataPNPGeneric(pipeline, result)
		pipeline.tagID.setInteger(result.getId())
		return drawnImg
	pipeline.hasTarget.setBoolean(False)
	return drawnImg

def pointToYawAndPitch(px: int, py: int) -> tuple[float, float]: # Converts a point in pixel system to a pitch and a yaw and returns that.
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
		pipeline.pixelX.setInteger(int(crosshair[0] - (resolutionWidth/2)))
		# Draw Stuff
		drawnImg = cv2.drawContours(drawnImg, mainContour, -1, color = (255, 0, 0), thickness = 2)
		drawnImg = cv2.rectangle(drawnImg, (x, y), (x + w, y + h), color = (0, 0, 255), thickness = 2)
		drawnImg = cv2.circle(drawnImg, center = crosshair, radius = 5, color = (0, 255, 0), thickness = -1)
		return drawnImg
	else:
		pipeline.hasTarget.setBoolean(False)
		return drawnImg
	   
def setupCameraConstants() -> None:
	global horizontalFOVRad, verticalFOVRad, cameraMatrix
	scale: float = (resolutionWidth / currentCameraType.resolutionWidth)
	horizontalFOVRad = getHorizontalFOVRad(resolutionWidth, resolutionHeight, currentCameraType.diagonalFOV)
	verticalFOVRad = getVerticalFOVRad(resolutionWidth, resolutionHeight, currentCameraType.diagonalFOV)
	print("Scale: " + str(scale))
	cameraMatrix = np.array([
		[currentCameraType.fx * scale, 0, currentCameraType.cx * scale],
		[0, currentCameraType.fy * scale, currentCameraType.cy * scale],
		[0, 0, 1]
	])
	
def main() -> None: # Image proccessing user code
	CameraServer.enableLogging()
	proccessedStream = CameraServer.putVideo("Proccessed Video", resolutionWidth, resolutionHeight)
	originalStream = CameraServer.putVideo("Original Video", resolutionWidth, resolutionHeight)
	mainTable.getEntry("Current Pipeline").setInteger(tagPipeline.pipelineIndex.getInteger(0))
	setupCameraConstants()
	print("Resolution: " + str(resolutionWidth) + "x" + str(resolutionHeight))
	print("Camera Matrix: \n" + str(cameraMatrix))
	print("Distortion Coefficients: " + str(currentCameraType.distCoeffs))
	# loop forever
	while True:
		index = mainTable.getEntry("Current Pipeline").getInteger(0)
		ret, frame = camera.read()
		if not ret:
			continue
		frame: Mat
		originalImg = frame.copy()
		# * Undistort the camera's image
		if currentCameraType is wideangleCam:
			frame = cv2.undistort(frame, cameraMatrix, currentCameraType.distCoeffs)
		drawnImg = frame.copy()
		for pipeline in pipelines:
			if (pipeline.pipelineIndex.getInteger(0) == index):
				if (pipeline.type == Pipeline.Type.COLOR):
					drawnImg = colorPipeline(frame, drawnImg, pipeline)
				elif (pipeline.type == Pipeline.Type.APRILTAG):
					drawnImg = aprilTagPipeline(frame, drawnImg, pipeline)
			else:
				pipeline.hasTarget.setBoolean(False)
		drawnImg = cv2.circle(drawnImg, (int(resolutionWidth/2), int(resolutionHeight/2)), 5, (255, 255, 255), -1)
		try:
			originalStream.putFrame(originalImg) # Stream Video
			proccessedStream.putFrame(drawnImg) # Stream Video
		except Exception as exception:
			print("Display Exception: " + exception)
		cv2.waitKey(10)

if __name__ == "__main__":
	if (onRPI()):
		if len(sys.argv) >= 2:
			configFile = sys.argv[1]

		# read configuration
		if not readConfig():
			sys.exit(1)

	# start NetworkTables
	ntinst = NetworkTableInstance.getDefault()
	arg = ""
	if len(sys.argv) >= 2:
		arg = str(sys.argv[1])
	if server or (not onRPI() and arg != "client"):
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
			pass
			# startSwitchedCamera(config)
	else:
		print("Running On Desktop")
		startCameraDesktop()
	# start opencv
	main()


