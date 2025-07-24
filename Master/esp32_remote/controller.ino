#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "esp_wifi.h"

// Add these includes at the top
#include <vector>
#include <cmath>



int prevBtn4State = HIGH;  // Assuming active-low button
unsigned long lastProfileChange = 0;
const unsigned long profileDebounceDelay = 200;  // ms

// Joystick pins
const int xPin = 34;
const int yPin = 35;
const int swPin = 32;
#define Btn3 27
#define Btn4 26
bool showDetailedData = false;  // Toggle between simple and detailed display

int bpin = 25;
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C  // Most common I2C address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



bool isAlgoMode = false;
bool prevBtnState = 0;
bool prevBtn2State = 0;


#define BUTTON_PIN 14  // Use any suitable GPIO pin

// Slave MAC Address
uint8_t slaveMac[] = { 0xE8, 0xDB, 0x84, 0xC0, 0xDE, 0x41 };

// AP credentials
const char *ssid = "A.X.I.O.N.";
const char *password = "Arpit@123";
// internet Connect credentials
const char *ssid_STA = "Naitik";
const char *password_STA = "Arpit@123";
// const int *wifiChannel=1;

// Web server and WebSocket server
WebServer server(80);
WebSocketsServer webSocket(81);

















char webpageT2[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Rover Live Mapper</title>
  <style>
    /* Container and layout styling - ALL SCREENS */
    body {
      background-color: #111;
      color: #0f0;
      font-family: "Courier New", Courier, monospace;
      margin: 0;
      padding: 0;
      overflow-x: hidden;
      overflow-y: auto;
      min-height: 100vh;
    }

    .container {
      width: 95%;
      max-width: 1200px;
      margin: 10px auto;
      text-align: center;
    }

    h1 {
      margin: 10px 0;
      font-size: clamp(1.2rem, 3vw, 1.8rem);
      color: #0f0;
      text-shadow: 0 0 5px #0f0;
    }

    /* Controls styling - ALL SCREENS */
    .controls {
      margin: 15px 0;
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 8px;
    }

    .controls button {
      background-color: #222;
      color: #0f0;
      border: 1px solid #0f0;
      padding: 8px 16px;
      font-size: clamp(0.8rem, 2vw, 1rem);
      border-radius: 5px;
      box-shadow: 0 0 10px rgba(0, 255, 0, 0.5);
      transition: all 0.3s ease;
      cursor: pointer;
      min-width: 100px;
    }

    .controls button:hover {
      box-shadow: 0 0 20px rgba(0, 255, 0, 0.8);
      transform: translateY(-2px);
    }

    /* Main content layout - ALL SCREENS */
    .main-content {
      display: flex;
      flex-direction: column;
      gap: 20px;
      padding: 10px;
    }

    /* Canvas styling - ALL SCREENS */
    #mapCanvas {
      background-color: #000;
      border: 2px solid #0f0;
      width: 100%;
      height: 60vh;
      margin-bottom: 20px;
      user-select: none;
      -webkit-user-select: none;
      touch-action: none;
    }

    /* Compass styling */
    .compass {
      position: absolute;
      top: 10px;
      right: 10px;
      width: 80px;
      height: 80px;
      z-index: 100;
    }

    .compass-arrow {
      width: 0;
      height: 0;
      border-left: 20px solid transparent;
      border-right: 20px solid transparent;
      border-bottom: 40px solid #f00;
      margin: 0 auto;
      transition: transform 0.3s;
    }

    .compass-label {
      text-align: center;
      color: #0f0;
      font-weight: bold;
      margin-top: 5px;
    }

    /* Sensor panel styling - ALL SCREENS */
    #sensorPanel {
      background-color: transparent;
      padding: 15px;
      width: 100%;
      margin: 20px auto;
      overflow-y: auto;
      max-height: 40vh;
    }

    .map-wrapper {
      position: relative;
      display: inline-block;
    }

    #sensorPanel div {
      margin-bottom: 12px;
      font-size: clamp(0.8rem, 1.5vw, 1rem);
      word-break: break-word;
    }

    /* PC-ONLY VIEW CHANGES (min-width: 1200px) */
    @media (min-width: 1200px) {
      .container {
        max-width: 100%;
        margin: 0 auto;
        padding: 0 20px;
      }

      .main-content {
        position: relative;
        display: flex;
        justify-content: center;
        align-items: center;
        height: auto;
      }

      #mapCanvas {
        width: 95vw;
        height: 80vh;
        border: 2px solid #0f0;
        box-shadow: 0 0 10px rgba(0, 255, 0, 0.3);
        display: block;
      }

      #sensorPanel {
        position: absolute;
        top: 30px;
        right: 0px;
        width: 280px;
        background-color: transparent;
        padding: 14px;
        z-index: 10;
        overflow-y: auto;
        color: #0f0;
      }

      .D h1 {
        font-size: 1.2rem;
        margin: 0;
        padding: 5px;
        background-color: rgba(0,0,0,0.7);
        border-radius: 5px;
        text-align: center;
      }

      .D { 
        position: absolute;
        top: 30px;
        left:20px;
        width: 280px;
        background-color: transparent;
        padding: 14px;
        z-index: 10;
        overflow-y: auto;
        color: #0f0;    
      }
    }
    
    /* TABLET VIEW */
    @media (min-width: 768px) and (max-width: 1199px) {
      #mapCanvas {
        width: 95%;
        height: 70vh;
        margin-bottom: 0;
      }

      #sensorPanel {
        width: 95%;
        max-height: 25vh;
        margin: 20px auto;
      }
    }

    /* MOBILE VIEW */
    @media (max-width: 767px) {
      .mobile-info {
        display: block;
        color: #0f0;
        font-size: 0.8rem;
        margin-top: 5px;
        text-align: center;
      }

      #mapCanvas {
        height: 50vh;
      }

      #sensorPanel {
        max-height: 30vh;
      }
      
      body {
        overflow-y: scroll;
        -webkit-overflow-scrolling: touch;
      }
      
      #mapCanvas {
        touch-action: pan-x pan-y;
      }
      
      #sensorPanel {
        overflow-y: auto;
        -webkit-overflow-scrolling: touch;
        max-height: 40vh !important;
      }
    }

    /* SMALL MOBILE VIEW */
    @media (max-width: 480px) {
      .controls {
        flex-direction: column;
        align-items: center;
      }

      .controls button {
        width: 80%;
        margin: 3px 0;
      }

      #sensorPanel {
        max-height: 40vh;
      }

      #sensorPanel div {
        font-size: 0.9rem;
      }
    }

    #followBtn {
      background-color: #00aa00;
      color: white;
    }

    #followBtn.off {
      background-color: #aa0000;
    }
  </style>
</head>
<body onload="load()">
  <div class="container">
    <h1>Rover Live Mapper</h1>

    <!-- Compass element -->
    <div class="compass">
      <div class="compass-arrow"></div>
      <div class="compass-label">N</div>
    </div>

    <div class="controls">
      <button id="startBtn">Start</button>
      <button id="resetBtn">Reset</button>
      <button id="zoomInBtn">Zoom In</button>
      <button id="zoomOutBtn">Zoom Out</button>
      <button id="followBtn">Auto-Follow: ON</button>
    </div>

    <p class="mobile-info">A project by Naitik Srivastava</p>

    <div class="main-content">
      <div class="map-wrapper">
        <canvas id="mapCanvas" width="800" height="600"></canvas>
        <div id="sensorPanel">
          <div>Yaw: <span id="yawValue">0</span>°</div>
          <div>Distance: <span id="distanceValue">0</span></div>
          <div>Front: <span id="frontValue">0</span></div>
          <div>Left: <span id="leftValue">0</span></div>
          <div>Right: <span id="rightValue">0</span></div>
        </div>
        <div class="D"><h1 id="D">HEADING DIRECTION</h1></div>
      </div>
    </div>
  </div>
</body>
<script>
  // State variables
  let realYawDeg = 0;
  let realleftDist = 0;
  let realrightDist = 0;
  let realfrontDist = 0;
  let realDist = 0;
  let realPosX = 0;
  let realPosY = 0;
  let frameCount = 0;
  let autoFollow = true;
  let smoothPanX = 0;
  let smoothPanY = 0;
  let webSocket;
  let isDragging = false;
  let lastPanX = 0;
  let lastPanY = 0;
  let panOffsetX = 0;
  let panOffsetY = 0;
  let scale = 1;
  let touchStartDistance = 0;
  let isPinching = false;
  let DIS = 0;
  let prevDIS = 0;
  let Mcmd = 0;
  let prevLeftDist = 0;
  let prevRightDist = 0;
  let totalDistance = 0;
  let lastMovementState = 5; // STOPPED
  let lastYaw = 0;
  
  // Movement states
  const MOVEMENT_STATES = {
    FORWARD: 1,
    BACKWARD: 2,
    TURN_LEFT: 3,
    TURN_RIGHT: 4,
    STOPPED: 5
  };

  // Physical parameters
  const WHEEL_BASE = 16; // cm (distance between wheels)
  const CM_PER_UNIT = 25; // cm per map unit
  const TURN_THRESHOLD = 15; // Degrees

  // Segment tracking
  let segments = [];
  let currentSegment = {
    startX: 0, 
    startY: 0, 
    startDistance: 0, 
    lastYaw: 0,
    movementType: MOVEMENT_STATES.STOPPED,
    distance: 0
  };

  // Canvas variables
  const canvas = document.getElementById("mapCanvas");
  const ctx = canvas.getContext("2d");
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;
  let posX = 0, posY = 0;
  let yaw = 0;
  let distance = 0;
  let cellSize = 20;
  let pathPoints = [{ x: posX, y: posY }];
  let obstacles = [];
  let simInterval = null;
  
  // Zoom limits
  const MIN_CELL_SIZE = 5;
  const MAX_CELL_SIZE = 50;

  // UI elements
  const startBtn = document.getElementById("startBtn");
  const resetBtn = document.getElementById("resetBtn");
  const zoomInBtn = document.getElementById("zoomInBtn");
  const zoomOutBtn = document.getElementById("zoomOutBtn");
  const yawValueElem = document.getElementById("yawValue");
  const distanceValueElem = document.getElementById("distanceValue");
  const frontValueElem = document.getElementById("frontValue");
  const leftValueElem = document.getElementById("leftValue");
  const rightValueElem = document.getElementById("rightValue");

  function load() {
    webSocket = new WebSocket("ws://" + window.location.hostname + ":81/");

    webSocket.onmessage = function (event) {
      try {
        const data = JSON.parse(event.data);
        console.log(data);

        // Update sensor values
        realYawDeg = data.Gx;
        realleftDist = data.distance2;
        realrightDist = data.distance3;
        realfrontDist = data.RadarDist;
        DIS = data.Distance;
        Mcmd = data.Manualcmd;

        // Update sensor panel
        document.getElementById("sensorPanel").innerHTML =
          "Temperature: " + data.Temp + " &#176;C<br>" +
          "Humidity: " + data.Humid + " %<br>" +
          "Pressure: " + data.Pressure + " Pa<br>" +
          "Altitude: " + data.Altitude + " m<br>" +
          "Battery Voltage: " + data.volt + " Volts<br>" +
          "Front Clearance: " + data.distance1 + " cm<br>" +
          "(RadarAngle, RadarDist): (" + data.angle + "&#176;, " + data.RadarDist + " cm)<br>" +
          "BIO SIGNS: " + (data.Pir ? "BIO Signatures Detected" : "No Bio Signatures") + "<br>" +
          "Left Clearance: " + data.distance2 + " cm<br>" +
          "Right Clearance: " + data.distance3 + " cm<br>" +
          "Acceleration: (" + data.AccX + ", " + data.AccY + ", " + data.AccZ + ")<br>" +
          "Yaw: " + data.Gx + "&#176;<br>" +
          "Current-Direction: " + data.State + "<br>" +
          "Algo-Direction: " + data.S + "<br>" +
          "Fire Detector: " + data.FireD + "<br>" +
          "IR1: " + data.ir1 + "<br>" +
          "IR2: " + data.ir2;
      } catch (e) {
        console.error("Error parsing WebSocket data:", e);
      }
    };

    // Add direction display
    updateDirectionDisplay();
    initCanvas();
  }

  function initCanvas() {
    // Touch event listeners
    canvas.addEventListener("touchstart", handleTouchStart, { passive: false });
    canvas.addEventListener("touchmove", handleTouchMove, { passive: false });
    canvas.addEventListener("touchend", handleTouchEnd);
    canvas.addEventListener("touchcancel", handleTouchEnd);

    // Mouse event listeners
    canvas.addEventListener("mousedown", handleMouseDown);
    canvas.addEventListener("mousemove", handleMouseMove);
    canvas.addEventListener("mouseup", handleMouseUp);
    canvas.addEventListener("mouseleave", handleMouseUp);

    // Auto-follow button
    document.getElementById("followBtn").addEventListener("click", function () {
      autoFollow = !autoFollow;
      this.textContent = autoFollow ? "Auto-Follow: ON" : "Auto-Follow: OFF";
      this.classList.toggle("off", !autoFollow);

      if (autoFollow) {
        smoothPanX = panOffsetX;
        smoothPanY = panOffsetY;
      }
    });

    // Button event listeners
    startBtn.addEventListener("click", startSimulation);
    resetBtn.addEventListener("click", resetSimulation);
    zoomInBtn.addEventListener("click", zoomIn);
    zoomOutBtn.addEventListener("click", zoomOut);

    // Initial render
    render();
  }

  function clearCanvas() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }

  // FIXED GRID FUNCTION
  function drawGrid() {
    ctx.strokeStyle = "rgba(0, 200, 0, 0.3)"; // More visible color
    ctx.lineWidth = 1;

    // Vertical grid lines
    const startX = panOffsetX % cellSize;
    for (let x = startX; x < canvas.width; x += cellSize) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, canvas.height);
      ctx.stroke();
    }

    // Horizontal grid lines
    const startY = panOffsetY % cellSize;
    for (let y = startY; y < canvas.height; y += cellSize) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();
    }
  }

  function drawPath() {
    if (pathPoints.length < 2) return;
    
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    const startPt = pathPoints[0];
    ctx.moveTo(
      centerX + startPt.x * cellSize + panOffsetX,
      centerY - startPt.y * cellSize + panOffsetY
    );
    
    for (let i = 1; i < pathPoints.length; i++) {
      const pt = pathPoints[i];
      ctx.lineTo(
        centerX + pt.x * cellSize + panOffsetX,
        centerY - pt.y * cellSize + panOffsetY
      );
    }
    
    ctx.strokeStyle = "#0f0";
    ctx.stroke();
  }

  function drawObstacles() {
    ctx.fillStyle = "#f00";
    obstacles.forEach((obs) => {
      const drawX = centerX + obs.x * cellSize + panOffsetX;
      const drawY = centerY - obs.y * cellSize + panOffsetY;
      const size = cellSize / 2;
      
      // Draw obstacle
      ctx.fillRect(drawX - size / 2, drawY - size / 2, size, size);
      
      // Draw distance text if zoomed in enough
      if (cellSize > 24 && cellSize < 51) {
        ctx.fillStyle = "#fff";
        ctx.font = "bold 11px Arial";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`${obs.distance.toFixed(0)}`, drawX, drawY);
        ctx.fillStyle = "#f00";
      }
    });
  }   

  function drawRover() {
    // Draw permanent blue starting point
    ctx.fillStyle = "#00f";
    ctx.beginPath();
    ctx.arc(
      centerX + panOffsetX,
      centerY + panOffsetY,
      cellSize * 0.3,
      0,
      Math.PI * 2
    );
    ctx.fill();

    ctx.strokeStyle = "#00f";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(
      centerX + panOffsetX,
      centerY + panOffsetY,
      cellSize * 0.7,
      0,
      Math.PI * 2
    );
    ctx.stroke();

    // Draw the rover
    const radius = cellSize * 0.3;
    const x = centerX + posX * cellSize + panOffsetX;
    const y = centerY - posY * cellSize + panOffsetY;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(yaw);

    ctx.fillStyle = "#ff0";
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = "#0f0";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, radius + 8, 0, Math.PI * 2);
    ctx.stroke();

    // Draw direction indicator
    const indicatorSize = cellSize * 0.5;
    ctx.fillStyle = "#f00";
    ctx.beginPath();
    ctx.moveTo(0, -radius - 5);
    ctx.lineTo(-indicatorSize/2, -radius - 5 - indicatorSize);
    ctx.lineTo(indicatorSize/2, -radius - 5 - indicatorSize);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }

  function updateDirectionDisplay() {
    const D = document.getElementById("D");
    const heading = (360 - ((realYawDeg % 360) + 360) % 360) % 360;
    
    if (heading >= 315 || heading < 45) D.innerText = "NORTH";
    else if (heading >= 45 && heading < 135) D.innerText = "EAST";
    else if (heading >= 135 && heading < 225) D.innerText = "SOUTH";
    else D.innerText = "WEST";
  }

  function updateCompass() {
    const arrow = document.querySelector('.compass-arrow');
    const label = document.querySelector('.compass-label');
    const headings = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    
    // Convert yaw to compass heading
    const degrees = (360 - ((yaw * 180 / Math.PI) % 360)) % 360;
    const index = Math.round(degrees / 45) % 8;
    
    // Update display
    arrow.style.transform = `rotate(${degrees}deg)`;
    label.textContent = headings[index];
  }

  function updateSensors() {
    // Calculate actual movement since last update
    const deltaLeft = (realleftDist - prevLeftDist) / CM_PER_UNIT;
    const deltaRight = (realrightDist - prevRightDist) / CM_PER_UNIT;
    
    // Store current values for next update
    prevLeftDist = realleftDist;
    prevRightDist = realrightDist;

    // Handle different movement states
    switch(Mcmd) {
      case MOVEMENT_STATES.FORWARD:
        // Forward movement - use average of both wheels
        const moveDist = (deltaLeft + deltaRight) / 2;
        posX += moveDist * Math.cos(yaw);
        posY += moveDist * Math.sin(yaw);
        distance += moveDist;
        break;
        
      case MOVEMENT_STATES.BACKWARD:
        // Backward movement - negative distance
        const reverseDist = (deltaLeft + deltaRight) / 2;
        posX -= reverseDist * Math.cos(yaw);
        posY -= reverseDist * Math.sin(yaw);
        distance += reverseDist;
        break;
        
      case MOVEMENT_STATES.TURN_LEFT:
      case MOVEMENT_STATES.TURN_RIGHT:
        // Pure rotation - only update yaw, not position
        const rotation = (deltaRight - deltaLeft) / WHEEL_BASE;
        yaw += rotation;
        // Normalize angle to [-π, π]
        yaw = (yaw + Math.PI) % (2 * Math.PI); 
        if (yaw < 0) yaw += 2 * Math.PI;
        yaw -= Math.PI;
        break;
        
      default:
        // No movement or unknown state
        break;
    }

    // Only add path point if actually moved (not just turning)
    if (Mcmd === MOVEMENT_STATES.FORWARD || Mcmd === MOVEMENT_STATES.BACKWARD) {
      if (frameCount++ % 3 === 0) {
        pathPoints.push({ x: posX, y: posY });
      }
    }

    // Update sensor displays
    yawValueElem.textContent = ((yaw * 180 / Math.PI + 360) % 360).toFixed(1);
    distanceValueElem.textContent = (distance * CM_PER_UNIT).toFixed(1);
    frontValueElem.textContent = realfrontDist.toFixed(1);
    leftValueElem.textContent = realleftDist.toFixed(1);
    rightValueElem.textContent = realrightDist.toFixed(1);

    // Update obstacles with direction awareness
    updateObstacles();

    // Auto-follow if enabled
    if (autoFollow) {
      updateAutoFollow();
    }
  }

  function updateObstacles() {
    obstacles = [];
    
    // Front obstacle (direction-sensitive)
    const frontDirX = Math.cos(yaw);
    const frontDirY = Math.sin(yaw);
    
    if (realfrontDist < 300 && realfrontDist > 2) {
      obstacles.push({
        x: posX + (realfrontDist / CM_PER_UNIT) * frontDirX,
        y: posY + (realfrontDist / CM_PER_UNIT) * frontDirY,
        distance: realfrontDist
      });
    }
    
    // Left obstacle (perpendicular to front)
    if (realleftDist < 300 && realleftDist > 2) {
      obstacles.push({
        x: posX + (realleftDist / CM_PER_UNIT) * Math.cos(yaw + Math.PI/2),
        y: posY + (realleftDist / CM_PER_UNIT) * Math.sin(yaw + Math.PI/2),
        distance: realleftDist
      });
    }
    
    // Right obstacle (perpendicular to front)
    if (realrightDist < 300 && realrightDist > 2) {
      obstacles.push({
        x: posX + (realrightDist / CM_PER_UNIT) * Math.cos(yaw - Math.PI/2),
        y: posY + (realrightDist / CM_PER_UNIT) * Math.sin(yaw - Math.PI/2),
        distance: realrightDist
      });
    }
  }

  function updateAutoFollow() {
    const targetPanX = -posX * cellSize;
    const targetPanY = posY * cellSize;
    
    smoothPanX += (targetPanX - smoothPanX) * 0.1;
    smoothPanY += (targetPanY - smoothPanY) * 0.1;
    
    panOffsetX = smoothPanX;
    panOffsetY = smoothPanY;
  }

  // Render the entire map
  function render() {
    clearCanvas();
    drawGrid(); // Fixed grid function
    drawObstacles();
    updateCompass();
    drawPath();
    drawRover();
  }

  // Touch event handlers
  function handleTouchStart(e) {
    if (autoFollow) {
      autoFollow = false;
      document.getElementById("followBtn").textContent = "Auto-Follow: OFF";
      document.getElementById("followBtn").classList.add("off");
    }

    if (e.touches.length === 1) {
      isDragging = true;
      lastPanX = e.touches[0].clientX;
      lastPanY = e.touches[0].clientY;
      e.preventDefault();
    } else if (e.touches.length === 2) {
      isPinching = true;
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      touchStartDistance = Math.sqrt(dx * dx + dy * dy);
      e.preventDefault();
    }
  }

  function handleTouchMove(e) {
    if (isDragging && e.touches.length === 1) {
      const deltaX = e.touches[0].clientX - lastPanX;
      const deltaY = e.touches[0].clientY - lastPanY;

      panOffsetX += deltaX;
      panOffsetY += deltaY;

      lastPanX = e.touches[0].clientX;
      lastPanY = e.touches[0].clientY;

      render();
      e.preventDefault();
    } else if (isPinching && e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      const touchDistance = Math.sqrt(dx * dx + dy * dy);

      if (touchStartDistance > 0) {
        const zoomFactor = touchDistance / touchStartDistance;
        const newCellSize = cellSize * zoomFactor;

        if (newCellSize > MIN_CELL_SIZE && newCellSize < MAX_CELL_SIZE) {
          cellSize = newCellSize;
        }

        touchStartDistance = touchDistance;
        render();
      }
      e.preventDefault();
    }
  }

  function handleTouchEnd(e) {
    isDragging = false;
    isPinching = false;
  }

  // Mouse event handlers
  function handleMouseDown(e) {
    if (autoFollow) {
      autoFollow = false;
      document.getElementById("followBtn").textContent = "Auto-Follow: OFF";
      document.getElementById("followBtn").classList.add("off");
    }
    isDragging = true;
    lastPanX = e.clientX;
    lastPanY = e.clientY;
    e.preventDefault();
  }

  function handleMouseMove(e) {
    if (isDragging) {
      const deltaX = e.clientX - lastPanX;
      const deltaY = e.clientY - lastPanY;

      panOffsetX += deltaX;
      panOffsetY += deltaY;

      lastPanX = e.clientX;
      lastPanY = e.clientY;

      render();
    }
  }

  function handleMouseUp() {
    isDragging = false;
  }

  // Start the simulation loop
  function startSimulation() {
    if (simInterval) return;
    simInterval = setInterval(() => {
      updateSensors();
      render();
    }, 100);
  }

  // Reset everything to initial state
  function resetSimulation() {
    clearInterval(simInterval);
    simInterval = null;
    posX = 0;
    posY = 0;
    panOffsetX = 0;
    panOffsetY = 0;
    prevLeftDist = 0;
    prevRightDist = 0;
    autoFollow = true;
    document.getElementById("followBtn").textContent = "Auto-Follow: ON";
    document.getElementById("followBtn").classList.remove("off");
    smoothPanX = 0;
    smoothPanY = 0;
    totalDistance = 0;
    lastMovementState = MOVEMENT_STATES.STOPPED;
    segments = [];
    currentSegment = {
      startX: 0,
      startY: 0,
      startDistance: 0,
      lastYaw: 0,
      movementType: MOVEMENT_STATES.STOPPED,
      distance: 0
    };
    yaw = 0;
    distance = 0;
    pathPoints = [{ x: posX, y: posY }];
    obstacles = [];
    
    // Reset displayed values
    yawValueElem.textContent = "0";
    distanceValueElem.textContent = "0";
    frontValueElem.textContent = "0";
    leftValueElem.textContent = "0";
    rightValueElem.textContent = "0";
    lastYaw = 0;
    
    render();
  }

  // Zoom controls
  function zoomIn() {
    if (cellSize < MAX_CELL_SIZE) {
      cellSize += 5;
      render();
    }
  }

  function zoomOut() {
    if (cellSize > MIN_CELL_SIZE) {
      cellSize -= 5;
      render();
    }
  }
</script>
</body>
</html>

)=====";


























// char webpage[] PROGMEM = R"=====(
// <!DOCTYPE html>
// <html>
//   <head>
//     <meta charset="UTF-8" />
//     <title>Rover Live Mapper</title>
//     <style>
//       /* Container and layout styling */
// body {
//   background-color: #111;
//   color: #0f0;
//   font-family: "Courier New", Courier, monospace;
//   margin: 0;
//   padding: 0;
//   overflow-x: hidden;
// }

// .container {
//   width: 95%;
//   max-width: 1200px;
//   margin: 10px auto;
//   text-align: center;
// }

// h1 {
//   margin: 10px 0;
//   font-size: clamp(1.2rem, 3vw, 1.8rem);
//   color: #0f0;
//   text-shadow: 0 0 5px #0f0;
// }

// /* Controls styling */
// .controls {
//   margin: 15px 0;
//   display: flex;
//   flex-wrap: wrap;
//   justify-content: center;
//   gap: 8px;
// }

// .controls button {
//   background-color: #222;
//   color: #0f0;
//   border: 1px solid #0f0;
//   padding: 8px 16px;
//   font-size: clamp(0.8rem, 2vw, 1rem);
//   border-radius: 5px;
//   box-shadow: 0 0 10px rgba(0, 255, 0, 0.5);
//   transition: all 0.3s ease;
//   cursor: pointer;
//   min-width: 100px;
// }

// .controls button:hover {
//   box-shadow: 0 0 20px rgba(0, 255, 0, 0.8);
//   transform: translateY(-2px);
// }

// /* Main content layout */
// .main-content {
//   display: flex;
//   flex-direction: column;
//   gap: 20px;
//   padding: 10px;
// }

// @media (min-width: 768px) {
//   .main-content {
//     flex-direction: row;
//     justify-content: space-between;
//     align-items: flex-start;
//   }
// }

// /* Canvas styling */
// #mapCanvas {
//   background-color: #000;
//   border: 2px solid #0f0;
//   width: 100%;
//   height: 60vh;
//   margin-bottom: 20px;
//   user-select: none;
//     -webkit-user-select: none;
//     touch-action: none;
// }

// @media (min-width: 768px) {
//   #mapCanvas {
//     width: 95%;
//     height: 70vh;
//     margin-bottom: 0;
//   }
// }

// /* Sensor panel styling */
// #sensorPanel {
//   background-color: #222;
//   border: 1px solid #0f0;
//   border-radius: 8px;
//   box-shadow: 0 0 10px #0f0;
//   padding: 15px;
//   width: 100%;
//   height: auto;
//   max-height: 40vh;
//   overflow-y: auto;
//    /* Keep all your existing styles... */
//   position: absolute;
//   top: 89vh;
//   right: 33vw; /* Keep original right value */
//   transform: translateX(-10px); /* Negative moves right, positive moves left */
//   /* ... rest of your existing styles */
// }

// @media (min-width: 768px) {
//   #sensorPanel {
//     width: 32%;
//     height: 25vh;
//     max-height: 25vh;
//   }
// }

// #sensorPanel div {
//   margin-bottom: 12px;
//   font-size: clamp(0.8rem, 1.5vw, 1rem);
//   word-break: break-word;
// }

// /* Very small screens adjustments */
// @media (max-width: 480px) {
//   .controls {
//     flex-direction: column;
//     align-items: center;
//   }

//   .controls button {
//     width: 80%;
//     margin: 3px 0;
//   }

//   #sensorPanel div {
//     font-size: 0.9rem;
//   }
// }

// #followBtn {
//     background-color: #00aa00; /* Green when ON */
//     color: white;
// }

// #followBtn.off {
//     background-color: #aa0000; /* Red when OFF */
// }

// /* Large screens adjustments */
// @media (min-width: 1200px) {
//   .container {
//     max-width: 1400px;
//   }

//   #mapCanvas {
//     width: 70%;
//   }

//   #sensorPanel {
//     width: 28%;
//   }
// }
//     </style>
//   </head>
//   <body onload="load()">
//     <div class="container">
//       <h1>Rover Live Mapper</h1>
//       <div class="controls">
//         <button id="startBtn">Start</button>
//         <button id="resetBtn">Reset</button>
//         <button id="zoomInBtn">Zoom In</button>
//         <button id="zoomOutBtn">Zoom Out</button>
//         <button id="followBtn">Auto-Follow: ON</button>
//       </div>
//       <canvas id="mapCanvas" width="800" height="600"></canvas>
//       <div id="sensorPanel">
//         <div>Yaw: <span id="yawValue">0</span>°</div>
//         <div>Distance: <span id="distanceValue">0</span></div>
//         <div>Front: <span id="frontValue">0</span></div>
//         <div>Left: <span id="leftValue">0</span></div>
//         <div>Right: <span id="rightValue">0</span></div>
//       </div>
//     </div>
//     <script>

// let realYawDeg = 0; // Store incoming Gx value (in degrees)
// let realleftDist = 0; // Store incoming Gx value (in degrees)
// let realrightDist = 0; // Store incoming Gx value (in degrees)
// let realfrontDist = 0; // Store incoming Gx value (in degrees)
// let realDist = 0; // Store incoming Gx value (in degrees)
// let realPosX = 0; // Store incoming Gx value (in degrees)
// let realPosY =0;
// let frameCount = 0;
// let autoFollow = true;  // Tracks if we should follow rover automatically
// let smoothPanX = 0;    // For smooth transitions
// let smoothPanY = 0;


//   var webSocket;

//   function load() {
//     webSocket = new WebSocket('ws://' + window.location.hostname + ':81/');

//     webSocket.onmessage = function(event) {
//       var data = JSON.parse(event.data);
//       console.log(data);

//       // Show movement label
//       let movementLabel = "";
//       switch (data.S) {
//         case "F": movementLabel = "Forward"; break;
//         case "B": movementLabel = "Backward"; break;
//         case "L": movementLabel = "Left"; break;
//         case "R": movementLabel = "Right"; break;
//         case "S": movementLabel = "Stop"; break;
//         case "G": movementLabel = "Ahead Left"; break;
//         case "H": movementLabel = "Back Left"; break;
//         case "I": movementLabel = "Ahead Right"; break;
//         case "J": movementLabel = "Back Right"; break;
//         default: movementLabel = "Unknown";
//       }

//    realYawDeg = data.Gx; // Store incoming Gx value (in degrees)
//    realleftDist = data.distance2; // Store incoming Gx value (in degrees)
//    realrightDist = data.distance3; // Store incoming Gx value (in degrees)
//    realfrontDist = data.RadarDist; // Store incoming Gx value (in degrees)
//    realDist = 0; // Store incoming Gx value (in degrees)
//    realPosX = data.PosX; // Store incoming Gx value (in degrees)
//    realPosY = data.PosY; // Store incoming Gx value (in degrees)


//       // Sensor data display
//       document.getElementById("sensorPanel").innerHTML =
//         "Temperature: " + data.Temp + " &#176;C<br>" +
//         "Humidity: " + data.Humid + " %<br>" +
//         "Pressure: " + data.Pressure + " Pa<br>" +
//         "Altitude: " + data.Altitude + " m<br>" +
//         "Battery Voltage: " + data.volt + "  Volts<br>" +
//         "Front Clearance: " + data.distance1 + " cm<br>" +
//         "(RadarAngle, RadarDist): (" + data.angle + "&#176;, " + data.RadarDist + " cm)<br>" +
//         "BIO SIGNS: " + (data.Pir ? "BIO Signatures Detected" : "No Bio Signatures") + "<br>" +
//         "Left Clearance: " + data.distance2 + " cm<br>" +
//         "Right Clearance: " + data.distance3 + " cm<br>" +
//         "Acceleration-X: " + data.AccX + "<br>" +
//         "Acceleration-Y: " + data.AccY + "<br>" +
//         "Acceleration-Z: " + data.AccZ + "<br>" +
//         "Yaw(Left or Right): " + data.Gx + "<br>" +
//         "Current-Direction: " + data.State + "<br>" +
//         "Algo-Direction: " + data.S + "<br>" +
//         "Fire Detector: " + data.FireD + "<br>" +
//         "IR1: " + data.ir1 + "<br>" +
//         "IR2: " + data.ir2;
//     };
//   }















//       let isDragging = false;
// let lastPanX = 0;
// let lastPanY = 0;
// let panOffsetX = 0;
// let panOffsetY = 0;




//       // Get canvas and its 2D context
//       const canvas = document.getElementById("mapCanvas");
//       const ctx = canvas.getContext("2d");


// // Panning event listeners
// canvas.addEventListener('mousedown', startPan);
// canvas.addEventListener('mousemove', handlePan);
// canvas.addEventListener('mouseup', endPan);
// canvas.addEventListener('mouseleave', endPan);

// // Touch support for mobile
// // Mouse events for desktop
// // Toggle button handler


// document.getElementById('followBtn').addEventListener('click', function() {
//     autoFollow = !autoFollow;
//     this.textContent = autoFollow ? "Auto-Follow: ON" : "Auto-Follow: OFF";
//     this.classList.toggle('off', !autoFollow);

//     if (autoFollow) {
//         smoothPanX = panOffsetX;  // Reset smooth pan to current position
//         smoothPanY = panOffsetY;
//     }
// });



// // Mouse down handler
// canvas.addEventListener('mousedown', (e) => {
//     if (autoFollow) {
//         autoFollow = false;
//         document.getElementById('followBtn').textContent = "Auto-Follow: OFF";
//     }
//     isDragging = true;
//     lastPanX = e.clientX;
//     lastPanY = e.clientY;
// });

// // Touch start handler
// canvas.addEventListener('touchstart', (e) => {
//     e.preventDefault();
//     if (autoFollow) {
//         autoFollow = false;
//         document.getElementById('followBtn').textContent = "Auto-Follow: OFF";
//     }
//     isDragging = true;
//     lastPanX = e.touches[0].clientX;
//     lastPanY = e.touches[0].clientY;
// });canvas.addEventListener('touchend', endPan);




//       // Pre-calc center of canvas for drawing coordinates
//       const centerX = canvas.width / 2;
//       const centerY = canvas.height / 2;

//       // UI elements from HTML
//       const startBtn = document.getElementById("startBtn");
//       const resetBtn = document.getElementById("resetBtn");
//       const zoomInBtn = document.getElementById("zoomInBtn");
//       const zoomOutBtn = document.getElementById("zoomOutBtn");
//       const yawValueElem = document.getElementById("yawValue");
//       const distanceValueElem = document.getElementById("distanceValue");
//       const frontValueElem = document.getElementById("frontValue");
//       const leftValueElem = document.getElementById("leftValue");
//       const rightValueElem = document.getElementById("rightValue");


//       // Simulation state variables
//       const CM_PER_UNIT = 10; // <-- Change this value to make each unit represent more or fewer cm

//       let posX = 0,
//       posY = 0; // Rover position (in map units, (0,0) at center)
//       let yaw = 0; // Rover orientation (radians)
//       let distance = 0; // Total distance traveled (map units)
//       let cellSize = 20; // Size of each grid cell (pixels)
//       let pathPoints = [{ x: posX, y: posY }]; // Array storing past rover positions
//       let obstacles = []; // Array storing obstacle positions {x,y}
//       let simInterval = null; // Reference for simulation loop interval

//       // Zoom limits to avoid too much zooming
//       const MIN_CELL_SIZE = 5;
//       const MAX_CELL_SIZE = 50;

//       // Clear entire canvas
//       function clearCanvas() {
//         ctx.clearRect(0, 0, canvas.width, canvas.height);
//       }

//       // Draw grid lines using current cellSize
//       function drawGrid() {
//     ctx.strokeStyle = "#333";
//     ctx.lineWidth = 1;

//     // Vertical grid lines
//     for (let x = panOffsetX % cellSize; x <= canvas.width; x += cellSize) {
//         ctx.beginPath();
//         ctx.moveTo(x, 0);
//         ctx.lineTo(x, canvas.height);
//         ctx.stroke();
//     }

//     // Horizontal grid lines
//     for (let y = panOffsetY % cellSize; y <= canvas.height; y += cellSize) {
//         ctx.beginPath();
//         ctx.moveTo(0, y);
//         ctx.lineTo(canvas.width, y);
//         ctx.stroke();
//     }
// }

// function drawPath() {
//     if (pathPoints.length < 2) return;

//     ctx.strokeStyle = "#0f0";
//     ctx.lineWidth = 2;
//     ctx.beginPath();

//     const startPt = pathPoints[0];
//     ctx.moveTo(
//         centerX + startPt.x * cellSize + panOffsetX,
//         centerY - startPt.y * cellSize + panOffsetY
//     );

//     for (let i = 1; i < pathPoints.length; i++) {
//         const pt = pathPoints[i];
//         ctx.lineTo(
//             centerX + pt.x * cellSize + panOffsetX,
//             centerY - pt.y * cellSize + panOffsetY
//         );
//     }
//     ctx.stroke();
// }

// function drawObstacles() {
//     ctx.fillStyle = "#f00";
//     obstacles.forEach((obs) => {
//         const drawX = centerX + obs.x * cellSize + panOffsetX;
//         const drawY = centerY - obs.y * cellSize + panOffsetY;
//         const size = cellSize / 2;
//         ctx.fillRect(drawX - size / 2, drawY - size / 2, size, size);
//     });
// }

// function drawRover() {
//     // Draw permanent blue starting point
//     ctx.fillStyle = "#00f";
//     ctx.beginPath();
//     ctx.arc(
//         centerX + panOffsetX,
//         centerY + panOffsetY,
//         cellSize * 0.3,
//         0,
//         Math.PI * 2
//     );
//     ctx.fill();

//     ctx.strokeStyle = "#00f";
//     ctx.lineWidth = 2;
//     ctx.beginPath();
//     ctx.arc(
//         centerX + panOffsetX,
//         centerY + panOffsetY,
//         cellSize * 0.7,
//         0,
//         Math.PI * 2
//     );
//     ctx.stroke();

//     // Draw the rover
//     const radius = cellSize * 0.3;
//     const x = centerX + posX * cellSize + panOffsetX;
//     const y = centerY - posY * cellSize + panOffsetY;

//     ctx.save();
//     ctx.translate(x, y);
//     ctx.rotate(yaw);

//     ctx.fillStyle = "#ff0";
//     ctx.beginPath();
//     ctx.arc(0, 0, radius, 0, Math.PI * 2);
//     ctx.fill();

//     ctx.strokeStyle = "#0f0";
//     ctx.lineWidth = 2;
//     ctx.beginPath();
//     ctx.arc(0, 0, radius + 8, 0, Math.PI * 2);
//     ctx.stroke();

//     ctx.restore();
// }      // Update simulation state: rover movement, sensor readings, and new obstacles
//       function updateSensors() {
//         // Simulate rotation and forward movement
//         // yaw = Math.random() * 360; // yaw value between 0 and 360
//         yaw = -(realYawDeg * Math.PI) / 180;

//         const moveStep = 0.5;
//         posX += (moveStep * Math.cos(yaw)) / CM_PER_UNIT;
//         posY += (moveStep * Math.sin(yaw)) / CM_PER_UNIT;
//         distance += moveStep / CM_PER_UNIT;

//         // Generate random sensor distance readings for demonstration
//         const frontDist = realfrontDist;
//         const leftDist = realleftDist;
//         const rightDist = realrightDist;

//         // Update sensor panel values in HTML
//         yawValueElem.textContent = (((yaw * 180) / Math.PI) % 360).toFixed(1);
//         distanceValueElem.textContent = distance.toFixed(1);
//         frontValueElem.textContent = frontDist.toFixed(1);
//         leftValueElem.textContent = leftDist.toFixed(1);
//         rightValueElem.textContent = rightDist.toFixed(1);



//         // Add current position to path
//         // Add current position to path (store only every 3rd point to save memory)
//         if (frameCount++ % 3 === 0) {  // Downsamples to 1/3 of points
//         pathPoints.push({ x: posX, y: posY });
//         }

//         // Never remove the first point, just trim excess (keeps origin visible)
//         if (pathPoints.length > 35000) {
//           pathPoints.shift();
//          }
//         // Add obstacles if sensor reading is below a threshold (simulate detection)
//         const threshold = 300;
//         if (frontDist < 128 && frontDist >2) {
//           obstacles.push({
//             x: posX + (frontDist / CM_PER_UNIT) * Math.cos(yaw),
//             y: posY + (frontDist / CM_PER_UNIT) * Math.sin(yaw),
//           });
//         }
//         if (leftDist < threshold && leftDist>1) {
//           obstacles.push({
//             x: posX + (leftDist / CM_PER_UNIT) * Math.cos(yaw + Math.PI / 2),
//             y: posY + (leftDist / CM_PER_UNIT) * Math.sin(yaw + Math.PI / 2),
//           });
//         }
//         if (rightDist < threshold && rightDist>1) {
//           obstacles.push({
//             x: posX + (rightDist / CM_PER_UNIT) * Math.cos(yaw - Math.PI / 2),
//             y: posY + (rightDist / CM_PER_UNIT) * Math.sin(yaw - Math.PI / 2),
//           });
//         }

//          // Auto-follow rover when enabled
//     if (autoFollow) {
//         // Calculate desired pan to keep rover centered
//         const targetPanX = -posX * cellSize;
//         const targetPanY = posY * cellSize;

//         // Smooth transition (adjust 0.1 for faster/slower follow)
//         smoothPanX += (targetPanX - smoothPanX) * 0.1;
//         smoothPanY += (targetPanY - smoothPanY) * 0.1;

//         // Apply the smooth pan
//         panOffsetX = smoothPanX;
//         panOffsetY = smoothPanY;
//     }




//       }





//       // Render the entire map (grid, path, obstacles, rover)
//       function render() {
//         clearCanvas();
//         drawGrid();
//         drawPath();
//         drawObstacles();
//         drawRover();
//       }



//       function startPan(e) {
//     isDragging = true;
//     lastPanX = e.clientX;
//     lastPanY = e.clientY;
// }

// function handlePan(e) {
//     if (!isDragging) return;

//     const deltaX = e.clientX - lastPanX;
//     const deltaY = e.clientY - lastPanY;

//     panOffsetX += deltaX;
//     panOffsetY += deltaY;

//     lastPanX = e.clientX;
//     lastPanY = e.clientY;

//     render();
// }

// function endPan() {
//     isDragging = false;
// }



//       // Start the simulation loop
//       function startSimulation() {
//         if (simInterval) return; // already running
//         simInterval = setInterval(() => {
//           updateSensors();
//           render();
//         }, 100); // update every 100 ms
//       }

//       // Reset everything to initial state
//       function resetSimulation() {
//         clearInterval(simInterval);
//         simInterval = null;
//         posX = 0;
//         posY = 0;
//         panOffsetX = 0;
//         panOffsetY = 0;
//         autoFollow = true;
//     document.getElementById('followBtn').textContent = "Auto-Follow: ON";
//     smoothPanX = 0;
//     smoothPanY = 0;
//             yaw = 0;
//         distance = 0;
//         pathPoints = [{ x: posX, y: posY }];
//         obstacles = [];
//         // Reset displayed values
//         yawValueElem.textContent = "0";
//         distanceValueElem.textContent = "0";
//         frontValueElem.textContent = "0";
//         leftValueElem.textContent = "0";
//         rightValueElem.textContent = "0";
//         render();
//       }

//       // Zoom controls adjust cellSize and redraw
//       function zoomIn() {
//         if (cellSize < MAX_CELL_SIZE) {
//           cellSize += 5;
//           render();
//         }
//       }
//       function zoomOut() {
//         if (cellSize > MIN_CELL_SIZE) {
//           cellSize -= 5;
//           render();
//         }
//       }

//       // Attach event listeners to buttons
//       startBtn.addEventListener("click", startSimulation);
//       resetBtn.addEventListener("click", resetSimulation);
//       zoomInBtn.addEventListener("click", zoomIn);
//       zoomOutBtn.addEventListener("click", zoomOut);


//       // Initial draw when page loads
//       render();
//     </script>
//   </ nload="load()">
// </html>



// )=====";




















// // HTML page
char webpageT[] PROGMEM = R"=====( 
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Rover Live Mapper</title>
  <style>
    /* Container and layout styling - ALL SCREENS */
    body {
      background-color: #111;
      color: #0f0;
      font-family: "Courier New", Courier, monospace;
      margin: 0;
      padding: 0;
      overflow-x: hidden;
      /* touch-action: none; */
      overflow-y: auto;
      min-height: 100vh;
    }

    .container {
      width: 95%;
      max-width: 1200px;
      margin: 10px auto;
      text-align: center;
    }

    h1 {
      margin: 10px 0;
      font-size: clamp(1.2rem, 3vw, 1.8rem);
      color: #0f0;
      text-shadow: 0 0 5px #0f0;
    }

    /* Controls styling - ALL SCREENS */
    .controls {
      margin: 15px 0;
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 8px;
    }

    .controls button {
      background-color: #222;
      color: #0f0;
      border: 1px solid #0f0;
      padding: 8px 16px;
      font-size: clamp(0.8rem, 2vw, 1rem);
      border-radius: 5px;
      box-shadow: 0 0 10px rgba(0, 255, 0, 0.5);
      transition: all 0.3s ease;
      cursor: pointer;
      min-width: 100px;
    }

    .controls button:hover {
      box-shadow: 0 0 20px rgba(0, 255, 0, 0.8);
      transform: translateY(-2px);
    }

    /* Main content layout - ALL SCREENS */
    .main-content {
      display: flex;
      flex-direction: column;
      gap: 20px;
      padding: 10px;
    }

    /* Canvas styling - ALL SCREENS */
    #mapCanvas {
      background-color: #000;
      border: 2px solid #0f0;
      width: 100%;
      height: 60vh;
      margin-bottom: 20px;
      user-select: none;
      -webkit-user-select: none;
      touch-action: none;
    }

    /* Sensor panel styling - ALL SCREENS */
    #sensorPanel {


  background-color: transparent;   /* make it see-through always */
  /* border: 1px solid #0f0; */
  /* border-radius: 8px; */
  /* box-shadow: 0 0 10px #0f0; */
  padding: 15px;
  width: 100%;
  margin: 20px auto;
  overflow-y: auto;
  max-height: 40vh;
  /* NO position here! */
    }

    .map-wrapper {
  position: relative;
  display: inline-block;
    }

    #sensorPanel div {
      margin-bottom: 12px;
      font-size: clamp(0.8rem, 1.5vw, 1rem);
      word-break: break-word;
    }

    /* PC-ONLY VIEW CHANGES (min-width: 1200px) */
    @media (min-width: 1200px) {
  .container {
    max-width: 100%;
    margin: 0 auto;
    padding: 0 20px;
  }

  .main-content {
    position: relative; /* Anchor for absolute overlay */
    display: flex;
    justify-content: center;
    align-items: center;
    height: auto;
  }

  #mapCanvas {
    width: 95vw;
    height: 80vh;
    border: 2px solid #0f0;
    box-shadow: 0 0 10px rgba(0, 255, 0, 0.3);
    display: block;
  }

  #sensorPanel {
    position: absolute;
    top: 30px;
    right: 0px;
    width: 280px;
    background-color: transparent; /* translucent background */
    /* backdrop-filter: blur(6px); */
    /* border: 1px solid rgba(0, 255, 0, 0.5);
    border-radius: 10px; */
    /* box-shadow: 0 0 10px rgba(0, 255, 0, 0.4); */
    padding: 14px;
    z-index: 10;
    overflow-y: auto;
    color: #0f0;
  }
  .D { 
    position: absolute;
    top: 30px;
left:20px;
    width: 280px;
    background-color: transparent; /* translucent background */
    /* backdrop-filter: blur(6px); */
    /* border: 1px solid rgba(0, 255, 0, 0.5);
    border-radius: 10px; */
    /* box-shadow: 0 0 10px rgba(0, 255, 0, 0.4); */
    padding: 14px;
    z-index: 10;
    overflow-y: auto;
    color: #0f0;    
  }
  
}
    /* TABLET VIEW (unchanged from your version) */
    @media (min-width: 768px) and (max-width: 1199px) {
      #mapCanvas {
        width: 95%;
        height: 70vh;
        margin-bottom: 0;
      }

      #sensorPanel {
        width: 95%;
        max-height: 25vh;
        margin: 20px auto;
      }
    }

    /* MOBILE VIEW (unchanged from your version) */
    @media (max-width: 767px) {
      .mobile-info {
        display: block;
        color: #0f0;
        font-size: 0.8rem;
        margin-top: 5px;
        text-align: center;
      }

      #mapCanvas {
        height: 50vh;
      }

      #sensorPanel {
        max-height: 30vh;
      }
      body {
    overflow-y: scroll;
    -webkit-overflow-scrolling: touch;
  }
  
  #mapCanvas {
    touch-action: pan-x pan-y;
  }
  
  #sensorPanel {
    overflow-y: auto;
    -webkit-overflow-scrolling: touch;
    max-height: 40vh !important;
  }
    }

    /* SMALL MOBILE VIEW (unchanged from your version) */
    @media (max-width: 480px) {
      .controls {
        flex-direction: column;
        align-items: center;
      }

      .controls button {
        width: 80%;
        margin: 3px 0;
      }

      #sensorPanel {
        max-height: 40vh;
      }

      #sensorPanel div {
        font-size: 0.9rem;
      }
    }

    #followBtn {
      background-color: #00aa00;
      color: white;
    }

    #followBtn.off {
      background-color: #aa0000;
    }
  </style>
</head>
<body onload="load()">
  <div class="container">
    <h1>Rover Live Mapper</h1>

    <div class="controls">
      <button id="startBtn">Start</button>
      <button id="resetBtn">Reset</button>
      <button id="zoomInBtn">Zoom In</button>
      <button id="zoomOutBtn">Zoom Out</button>
      <button id="followBtn">Auto-Follow: ON</button>
    </div>

    <p class="mobile-info">A project by Naitik Srivastava</p>

    <div class="main-content">
      <!-- Wrap canvas and sensorPanel together -->
      <div class="map-wrapper">
        <canvas id="mapCanvas" width="800" height="600"></canvas>
        <div id="sensorPanel">
          <div>Yaw: <span id="yawValue">0</span>°</div>
          <div>Distance: <span id="distanceValue">0</span></div>
          <div>Front: <span id="frontValue">0</span></div>
          <div>Left: <span id="leftValue">0</span></div>
          <div>Right: <span id="rightValue">0</span></div>
        </div>
        <div  class="D"> <h1 id="D">HEADING DIRECTION</h1>
        </div>
      </div>
    </div>
  </div>
</body>
<script>
      let realYawDeg = 0;
      let realleftDist = 0;
      let realrightDist = 0;
      let realfrontDist = 0;
      let realDist = 0;
      let realPosX = 0;
      let realPosY = 0;
      let frameCount = 0;
      let autoFollow = true;
      let smoothPanX = 0;
      let smoothPanY = 0;
      let webSocket;
      let isDragging = false;
      let lastPanX = 0;
      let lastPanY = 0;
      let panOffsetX = 0;
      let panOffsetY = 0;
      let scale = 1;
      let touchStartDistance = 0;
      let isPinching = false;
      let DIS=0;
      let prevDIS = 0;
      let Mcmd = 0; 


      function load() {
        webSocket = new WebSocket("ws://" + window.location.hostname + ":81/");

        webSocket.onmessage = function (event) {
          var data = JSON.parse(event.data);
          console.log(data);

          realYawDeg = data.Gx;
          realleftDist = data.distance2;
          realrightDist = data.distance3;
          realfrontDist = data.RadarDist;
          realDist = 0;
          realPosX = data.PosX;
          realPosY = data.PosY;
           DIS = data.Distance;
           Mcmd = data.Manualcmd;


            document.getElementById("sensorPanel").innerHTML =
              "Temperature: " +
              data.Temp +
              " &#176;C<br>" +
              "Humidity: " +
              data.Humid +
              " %<br>" +
              "Pressure: " +
              data.Pressure +
              " Pa<br>" +
              "Altitude: " +
              data.Altitude +
              " m<br>" +
              "Battery Voltage: " +
              data.volt +
              "  Volts<br>" +
              "RightWheelDist: " +
              data.PosX +
              "  cm<br>" +
              "LeftWheelDist: " +
              data.PosY +
              "  cm<br>" +
              "Front Clearance: " +
              data.distance1 +
              " cm<br>" +
              "(RadarAngle, RadarDist): (" +
              data.angle +
              "&#176;, " +
              data.RadarDist +
              " cm)<br>" +
              "BIO SIGNS: " +
              (data.Pir ? "BIO Signatures Detected" : "No Bio Signatures") +
              "<br>" +
              "Left Clearance: " +
              data.distance2 +
              " cm<br>" +
              "Right Clearance: " +
              data.distance3 +
              " cm<br>" +
              "Acceleration-X: " +
              data.AccX +
              "<br>" +
              "Acceleration-Y: " +
              data.AccY +
              "<br>" +
              "Acceleration-Z: " +
              data.AccZ +
              "<br>" +
              "Yaw(Left or Right): " +
              data.Gx +
              "<br>" +
              "Current-Direction: " +
              data.State +
              "<br>" +
              "Algo-Direction: " +
              data.S +
              "<br>" +
              "Fire Detector: " +
              data.FireD +
              "<br>" +
              "IR1: " +
              data.ir1 +
              "<br>" +
              "IR2: " +
              data.ir2;
          };
  

        initCanvas();
      }

      // Get canvas and its 2D context
      const canvas = document.getElementById("mapCanvas");
      const ctx = canvas.getContext("2d");

      // Pre-calc center of canvas for drawing coordinates
      const centerX = canvas.width / 2;
      const centerY = canvas.height / 2;

      // UI elements from HTML
      const startBtn = document.getElementById("startBtn");
      const resetBtn = document.getElementById("resetBtn");
      const zoomInBtn = document.getElementById("zoomInBtn");
      const zoomOutBtn = document.getElementById("zoomOutBtn");
      const yawValueElem = document.getElementById("yawValue");
      const distanceValueElem = document.getElementById("distanceValue");
      const frontValueElem = document.getElementById("frontValue");
      const leftValueElem = document.getElementById("leftValue");
      const rightValueElem = document.getElementById("rightValue");
   
    
    
    // Simulation state variables

// Segment tracking variables
let segments = []; // Stores completed path segments
let currentSegment = { // Tracks current straight segment
  startX: 0, 
  startY: 0, 
  startDistance: 0, 
  lastYaw: 0 
};
let lastYaw = 0; // Stores previous yaw for comparison



      const CM_PER_UNIT = 15;
      let posX = 0,
        posY = 0;
      let yaw = 0;
      let distance = 0;
      let cellSize = 20;
      let pathPoints = [{ x: posX, y: posY }];
      let obstacles = [];
      let simInterval = null;

      // Zoom limits to avoid too much zooming
      const MIN_CELL_SIZE = 5;
      const MAX_CELL_SIZE = 50;

      function initCanvas() {
        // Add touch event listeners
        canvas.addEventListener("touchstart", handleTouchStart, {
          passive: false,
        });
        canvas.addEventListener("touchmove", handleTouchMove, {
          passive: false,
        });
        canvas.addEventListener("touchend", handleTouchEnd);
        canvas.addEventListener("touchcancel", handleTouchEnd);

        // Mouse event listeners
        canvas.addEventListener("mousedown", handleMouseDown);
        canvas.addEventListener("mousemove", handleMouseMove);
        canvas.addEventListener("mouseup", handleMouseUp);
        canvas.addEventListener("mouseleave", handleMouseUp);

        // Auto-follow button
        document
          .getElementById("followBtn")
          .addEventListener("click", function () {
            autoFollow = !autoFollow;
            this.textContent = autoFollow
              ? "Auto-Follow: ON"
              : "Auto-Follow: OFF";
            this.classList.toggle("off", !autoFollow);

            if (autoFollow) {
              smoothPanX = panOffsetX;
              smoothPanY = panOffsetY;
            }
          });

        // Button event listeners
        startBtn.addEventListener("click", startSimulation);
        resetBtn.addEventListener("click", resetSimulation);
        zoomInBtn.addEventListener("click", zoomIn);
        zoomOutBtn.addEventListener("click", zoomOut);

        // Initial render
        render();
      }

      // Clear entire canvas
      function clearCanvas() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
      }

      // Draw grid lines using current cellSize
      function drawGrid() {
        ctx.strokeStyle = "#333";
        ctx.lineWidth = 1;

        // Vertical grid lines
        for (let x = panOffsetX % cellSize; x <= canvas.width; x += cellSize) {
          ctx.beginPath();
          ctx.moveTo(x, 0);
          ctx.lineTo(x, canvas.height);
          ctx.stroke();
        }

        // Horizontal grid lines
        for (let y = panOffsetY % cellSize; y <= canvas.height; y += cellSize) {
          ctx.beginPath();
          ctx.moveTo(0, y);
          ctx.lineTo(canvas.width, y);
          ctx.stroke();
        }
      }

      function drawPath() {
        if (pathPoints.length < 2) return;

        ctx.strokeStyle = "#0f0";
        ctx.lineWidth = 2;
        ctx.beginPath();

        const startPt = pathPoints[0];
        ctx.moveTo(
          centerX + startPt.x * cellSize + panOffsetX,
          centerY - startPt.y * cellSize + panOffsetY
        );

        for (let i = 1; i < pathPoints.length; i++) {
          const pt = pathPoints[i];
          ctx.lineTo(
            centerX + pt.x * cellSize + panOffsetX,
            centerY - pt.y * cellSize + panOffsetY
          );
        }
        ctx.stroke();
      }

      function drawObstacles() {
    ctx.fillStyle = "#f00";
    obstacles.forEach((obs) => {
        const drawX = centerX + obs.x * cellSize + panOffsetX;
        const drawY = centerY - obs.y * cellSize + panOffsetY;
        const size = cellSize / 2;
        
        // Draw the obstacle rectangle
        ctx.fillRect(drawX - size / 2, drawY - size / 2, size, size);
        if(cellSize>24 && cellSize<51) {
          
        // Draw distance text inside (new code)
        ctx.fillStyle = "#fff"; // Green text
        ctx.font = "bold 11px Arial"; // Small, readable font
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`${obs.distance.toFixed(0)}`, drawX, drawY);
        
        // Reset to red for next obstacle
        ctx.fillStyle = "#f00";}

    });
}   

function drawSegmentDistances() {
  const textPadding = 2; // Padding around text
  const minSegmentLength = 1.0; // Minimum segment length to display
  
  // Draw distances for completed segments
  segments.forEach(segment => {
    if (segment.distance < minSegmentLength) return;
    
    const midX = (segment.startX + segment.endX) / 2;
    const midY = (segment.startY + segment.endY) / 2;
    
    const canvasX = centerX + midX * cellSize + panOffsetX;
    const canvasY = centerY - midY * cellSize + panOffsetY;
    
    // Calculate text dimensions
    const text = segment.distance.toFixed(1)*10 + "cm";
    ctx.font = "bold " + Math.max(10, cellSize/3) + "px Arial";
    const textWidth = ctx.measureText(text).width;
    
    // Draw background rectangle
    ctx.fillStyle = "rgba(0, 0, 0, 0.7)";
    ctx.fillRect(
      canvasX - textWidth/2 - textPadding,
      canvasY - cellSize/4 - textPadding,
      textWidth + textPadding*2,
      cellSize/2 + textPadding*2
    );
    
    // Draw text
    ctx.fillStyle = "#0f0";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, canvasX, canvasY);
  });
  
  // Draw distance for current active segment
  const currentSegDistance = distance - currentSegment.startDistance;
  if (currentSegDistance > minSegmentLength) {
    const midX = (currentSegment.startX + posX) / 2;
    const midY = (currentSegment.startY + posY) / 2;
    
    const canvasX = centerX + midX * cellSize + panOffsetX;
    const canvasY = centerY - midY * cellSize + panOffsetY;
    
    // Calculate text dimensions
    const text = currentSegDistance.toFixed(1)*10 + "cm";
    ctx.font = "bold " + Math.max(10, cellSize/3) + "px Arial";
    const textWidth = ctx.measureText(text).width;
    
    // Draw background rectangle
    ctx.fillStyle = "rgba(0, 0, 0, 0.7)";
    ctx.fillRect(
      canvasX - textWidth/2 - textPadding,
      canvasY - cellSize/4 - textPadding,
      textWidth + textPadding*2,
      cellSize/2 + textPadding*2
    );
    
    // Draw text
    ctx.fillStyle = "#0f0";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, canvasX, canvasY);
  }
}

function drawRover() {
        // Draw permanent blue starting point
        ctx.fillStyle = "#00f";
        ctx.beginPath();
        ctx.arc(
          centerX + panOffsetX,
          centerY + panOffsetY,
          cellSize * 0.3,
          0,
          Math.PI * 2
        );
        ctx.fill();

        ctx.strokeStyle = "#00f";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(
          centerX + panOffsetX,
          centerY + panOffsetY,
          cellSize * 0.7,
          0,
          Math.PI * 2
        );
        ctx.stroke();

        // Draw the rover
        const radius = cellSize * 0.3;
        const x = centerX + posX * cellSize + panOffsetX;
        const y = centerY - posY * cellSize + panOffsetY;

        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(yaw);

        ctx.fillStyle = "#ff0";
        ctx.beginPath();
        ctx.arc(0, 0, radius, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = "#0f0";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(0, 0, radius + 8, 0, Math.PI * 2);
        ctx.stroke();

        ctx.restore();
      }

      // Update simulation state: rover movement, sensor readings, and new obstacles
      function updateSensors() {
// DIS+= 1; (this line remains as is from your original code)

// Initialize turning state variables if not defined
if (typeof isTurning === 'undefined') {
    var isTurning = false;
    var preTurnDIS = DIS;
}

// Initialize prevDIS if it's not set
if (typeof prevDIS === 'undefined') {
    prevDIS = DIS;
}

// Handle Mcmd 3/4 states
if (Mcmd === 3 || Mcmd === 4) {
    if (!isTurning) {
        preTurnDIS = prevDIS;  // Save DIS state before turning started
        isTurning = true;
    }
    prevDIS = DIS;  // Freeze position tracking during turns
} else if (isTurning) {
    prevDIS = preTurnDIS;  // Restore pre-turn DIS state
    isTurning = false;
}

// Convert yaw to radians (assuming realYawDeg is in degrees)
yaw = -(realYawDeg * Math.PI) / 180;

// Calculate distance moved since last update (in cm)
const deltaDIS = DIS - prevDIS;
prevDIS = DIS;  // Store current distance for next update

// Only process movement if not turning AND significant distance moved
if (!(Mcmd === 3 || Mcmd === 4)) {
    // Convert movement to position units
    const deltaDistance = deltaDIS / CM_PER_UNIT;

    // Only update position if rover has moved significantly (>1cm threshold)
    if (deltaDistance >= 0.1) {
        // Update position (using current yaw direction)
        posX += deltaDistance * Math.cos(yaw);
        posY += deltaDistance * Math.sin(yaw);
        distance += deltaDistance;

        // Segment detection logic (for path segmentation)
        const yawDeg = (yaw * 180) / Math.PI;
        const yawChange = Math.abs(yawDeg - lastYaw);

        // If significant direction change (>50 degrees)
        if (yawChange > 50) {
            // Finalize current segment if meaningful distance was traveled
            const segmentDistance = distance - currentSegment.startDistance;
            if (segmentDistance > 0.5) { // Minimum segment length threshold
                segments.push({
                    startX: currentSegment.startX,
                    startY: currentSegment.startY,
                    endX: posX,
                    endY: posY,
                    distance: segmentDistance
                });
            }
            
            // Start new segment
            currentSegment = {
                startX: posX,
                startY: posY,
                startDistance: distance,
                lastYaw: yawDeg
            };
        }
        
        // Update lastYaw for next comparison
        lastYaw = yawDeg;
    }
}
        // const moveStep = 0.5;
        // posX += (moveStep * Math.cos(yaw)) / CM_PER_UNIT;
        // posY += (moveStep * Math.sin(yaw)) / CM_PER_UNIT;
        // distance += moveStep / CM_PER_UNIT;






        // // const moveStep = 0.5;
        // posX = (DIS * Math.cos(yaw)) / CM_PER_UNIT;
        // posY = (DIS * Math.sin(yaw)) / CM_PER_UNIT;
        // distance = DIS / CM_PER_UNIT;

        // Ensure rover stays within bounds
        const frontDist = realfrontDist;
        const leftDist = realleftDist;
        const rightDist = realrightDist;
        // const frontDist = Math.random() * 200 + 50;
        // const leftDist = Math.random() * 200 + 50;
        // const rightDist = Math.random() * 200 + 50;

      //  const frontDist = 100;
      //   const leftDist = 75;
      //   const rightDist = 50;


        // Update sensor panel values in HTML
        yawValueElem.textContent = (((yaw * 180) / Math.PI) % 360).toFixed(1);
        distanceValueElem.textContent = distance.toFixed(1);
        frontValueElem.textContent = frontDist.toFixed(1);
        leftValueElem.textContent = leftDist.toFixed(1);
        rightValueElem.textContent = rightDist.toFixed(1);

        // Add current position to path
        if (frameCount++ % 3 === 0) {
          pathPoints.push({ x: posX, y: posY });
        }

        // Never remove the first point, just trim excess (keeps origin visible)
        if (pathPoints.length > 35000) {
          pathPoints.shift();
        }


        // Add obstacles if sensor reading is below a threshold (simulate detection)
        const threshold = 300;
        if (frontDist < 128 && frontDist > 2) {
          obstacles.push({
            x: posX + (frontDist / CM_PER_UNIT) * Math.cos(yaw),
            y: posY + (frontDist / CM_PER_UNIT) * Math.sin(yaw),
            distance: frontDist  // Add this line

          });
        }
        if (leftDist < threshold && leftDist > 1) {
          obstacles.push({
            x: posX + (leftDist / CM_PER_UNIT) * Math.cos(yaw + Math.PI / 2),
            y: posY + (leftDist / CM_PER_UNIT) * Math.sin(yaw + Math.PI / 2),
            distance: leftDist  // Add this line

          });
        }
        if (rightDist < threshold && rightDist > 1) {
          obstacles.push({
            x: posX + (rightDist / CM_PER_UNIT) * Math.cos(yaw - Math.PI / 2),
            y: posY + (rightDist / CM_PER_UNIT) * Math.sin(yaw - Math.PI / 2),
                distance: rightDist  // Add this line

          });
        }

        // Auto-follow rover when enabled
        if (autoFollow) {
          // Calculate desired pan to keep rover centered
          const targetPanX = -posX * cellSize;
          const targetPanY = posY * cellSize;

          // Smooth transition (adjust 0.1 for faster/slower follow)
          smoothPanX += (targetPanX - smoothPanX) * 0.1;
          smoothPanY += (targetPanY - smoothPanY) * 0.1;

          // Apply the smooth pan
          panOffsetX = smoothPanX;
          panOffsetY = smoothPanY;
        }
      }

      // Render the entire map (grid, path, obstacles, rover)
      function render() {
        clearCanvas();
        drawGrid();
        drawObstacles();  

        drawPath();
        drawSegmentDistances();

        drawRover();
      }

      // Touch event handlers
      function handleTouchStart(e) {
        if (autoFollow) {
          autoFollow = false;
          document.getElementById("followBtn").textContent = "Auto-Follow: OFF";
          document.getElementById("followBtn").classList.add("off");
        }

        if (e.touches.length === 1) {
          // Single touch - pan
          isDragging = true;
          lastPanX = e.touches[0].clientX;
          lastPanY = e.touches[0].clientY;
          e.preventDefault();
        } else if (e.touches.length === 2) {
          // Two touches - pinch to zoom
          isPinching = true;
          const dx = e.touches[0].clientX - e.touches[1].clientX;
          const dy = e.touches[0].clientY - e.touches[1].clientY;
          touchStartDistance = Math.sqrt(dx * dx + dy * dy);
          e.preventDefault();
        }
      }

      function handleTouchMove(e) {
        if (isDragging && e.touches.length === 1) {
          // Pan with one finger
          const deltaX = e.touches[0].clientX - lastPanX;
          const deltaY = e.touches[0].clientY - lastPanY;

          panOffsetX += deltaX;
          panOffsetY += deltaY;

          lastPanX = e.touches[0].clientX;
          lastPanY = e.touches[0].clientY;

          render();
          e.preventDefault();
        } else if (isPinching && e.touches.length === 2) {
          // Zoom with two fingers
          const dx = e.touches[0].clientX - e.touches[1].clientX;
          const dy = e.touches[0].clientY - e.touches[1].clientY;
          const touchDistance = Math.sqrt(dx * dx + dy * dy);

          if (touchStartDistance > 0) {
            const zoomFactor = touchDistance / touchStartDistance;
            const newCellSize = cellSize * zoomFactor;

            if (newCellSize > MIN_CELL_SIZE && newCellSize < MAX_CELL_SIZE) {
              cellSize = newCellSize;
            }

            touchStartDistance = touchDistance;
            render();
          }
          e.preventDefault();
        }
      }

      function handleTouchEnd(e) {
        isDragging = false;
        isPinching = false;
      }

    //      const D = document.getElementById("D");
    
    
    
    //     if(realYawDeg>283 && realYawDeg<285)  {
    //   D.innerText="FACING NORTH";
    // }

    

      // Mouse event handlers
      function handleMouseDown(e) {
        if (autoFollow) {
          autoFollow = false;
          document.getElementById("followBtn").textContent = "Auto-Follow: OFF";
          document.getElementById("followBtn").classList.add("off");
        }
        isDragging = true;
        lastPanX = e.clientX;
        lastPanY = e.clientY;
        e.preventDefault();
      }

      function handleMouseMove(e) {
        if (isDragging) {
          const deltaX = e.clientX - lastPanX;
          const deltaY = e.clientY - lastPanY;

          panOffsetX += deltaX;
          panOffsetY += deltaY;

          lastPanX = e.clientX;
          lastPanY = e.clientY;

          render();
        }
      }

      function handleMouseUp() {
        isDragging = false;
      }

      // Start the simulation loop
      function startSimulation() {
        if (simInterval) return; // already running
        simInterval = setInterval(() => {
          updateSensors();
          render();
        }, 100); // update every 100 ms
      }

      // Reset everything to initial state
      function resetSimulation() {
        clearInterval(simInterval);
        simInterval = null;
        posX = 0;
        posY = 0;
        panOffsetX = 0;
        panOffsetY = 0;
        autoFollow = true;
        document.getElementById("followBtn").textContent = "Auto-Follow: ON";
        document.getElementById("followBtn").classList.remove("off");
        smoothPanX = 0;
        smoothPanY = 0;
        yaw = 0;
        distance = 0;
        pathPoints = [{ x: posX, y: posY }];
        obstacles = [];
        // Reset displayed values
        yawValueElem.textContent = "0";
        distanceValueElem.textContent = "0";
        frontValueElem.textContent = "0";
        leftValueElem.textContent = "0";
        rightValueElem.textContent = "0";
        segments = [];
currentSegment = { startX: 0, startY: 0, startDistance: 0, lastYaw: 0 };
lastYaw = 0;
        render();
      }

      // Zoom controls adjust cellSize and redraw
      function zoomIn() {
        if (cellSize < MAX_CELL_SIZE) {
          cellSize += 5;
          render();
        }
      }

      function zoomOut() {
        if (cellSize > MIN_CELL_SIZE) {
          cellSize -= 5;
          render();
        }
      }
    </script>
  </body>
</html>
)=====";


// Data structures
#pragma pack(push, 1)
struct JoystickData {
  float x;
  float y;
  int sw;
  int Btn;
};
JoystickData data;


struct SlaveData {
  int Humid;
  float Temp;
  float Pressure;
  float Alt;
  float Voltage;
  int distance1, distance2, distance3;
  int ir1, ir2, Fire, angle;
  float X, Y, Z, Ax, Ay, Az;
  int RadarDist;  // <-- New
  int Pir;        // <-- New
  int mode;
  float PosX;
  float PosY;
  int ALGOCMD;
  int targetYaw;
  char cmd[4];  // enough for "FL", "BR", etc. + null terminator
};
SlaveData sData;
#pragma pack(pop)

// ----- BLINK LOGIC START -----
bool communicationSuccess = false;  // [ADDED]
unsigned long lastBlinkTime = 0;    // [ADDED]
bool ledState = false;              // [ADDED]
// ----- BLINK LOGIC END -----

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(SlaveData)) {
    memcpy(&sData, incomingData, len);
    communicationSuccess = true;  // [ADDED]
  }
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    communicationSuccess = true;  // [ADDED]
  } else {
    Serial.println("Delivery failed");
  }
}

void Serve_On_Page() {
  float Distance;
  int temp;
  if (sData.PosX >= sData.PosY) {
    Distance = sData.PosX;

  } else {
    Distance = sData.PosY;
  }


  if (data.x > 1900 && data.y < 1300) {
    // display.print("BR");
    temp = 9;
  } else if (data.x > 1900 && data.y > 1900) {
    // display.print("BL");
    temp = 8;

  } else if (data.x < 1300 && data.y < 1300) {
    // display.print("FR");
    temp = 7;

  } else if (data.x < 1300 && data.y > 1900) {
    // display.print("FL");
    temp = 6;

  } else if (data.x < 1300) {
    // display.print(" F");
    temp = 1;

  } else if (data.x > 1900) {
    // display.print(" B");
    temp = 2;

  } else if (data.y > 1900) {
    // display.print(" L");
    temp = 3;

  } else if (data.y < 1300) {
    // display.print(" R");
    temp = 4;

  } else {
    // display.print(" S");
    temp = 5;
  }



  String json = "{";
  json += "\"Temp\":" + String(sData.Temp) + ",";
  json += "\"Humid\":" + String(sData.Humid - 15) + ",";
  json += "\"volt\":" + String(sData.Voltage) + ",";
  json += "\"Pressure\":" + String(sData.Pressure) + ",";
  json += "\"Altitude\":" + String(sData.Alt) + ",";
  json += "\"distance1\":" + String(sData.distance1) + ",";
  json += "\"distance2\":" + String(sData.distance2) + ",";
  json += "\"distance3\":" + String(sData.distance3) + ",";
  json += "\"AccX\":" + String(sData.Ax) + ",";
  json += "\"AccY\":" + String(sData.Ay) + ",";
  json += "\"AccZ\":" + String(sData.Az) + ",";
  json += "\"Gx\":" + String(sData.X) + ",";
  json += "\"Gy\":" + String(sData.Y) + ",";
  json += "\"PosX\":" + String(sData.PosX) + ",";
  json += "\"PosY\":" + String(sData.PosY) + ",";
  json += "\"Distance\":" + String(Distance) + ",";
  json += "\"Manualcmd\":" + String(temp) + ",";
  json += "\"ALGOcmd\":" + String(sData.ALGOCMD) + ",";
  json += "\"Gz\":" + String(sData.Z) + ",";
  json += "\"angle\":" + String(sData.angle) + ",";
  json += "\"ir1\":" + String(sData.ir1) + ",";
  json += "\"ir2\":" + String(sData.ir2) + ",";
  json += "\"RadarDist\":" + String(sData.RadarDist) + ",";
  json += "\"Pir\":" + String(sData.Pir) + ",";
  json += "\"FireD\":" + String(sData.Fire);
  json += "}";
  webSocket.broadcastTXT(json);
  // Serial.println(sData.Voltage);
  // Serial.println(sData.RadarDist);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  // Optionally process received messages here
}

// void HTTP_handleRoot() {
//   server.send_P(200, "text/html", webpage);
// }
void testServe() {
  server.send_P(200, "text/html", webpageT);
}
void testV2Serve() {
  server.send_P(200, "text/html", webpageT2);
}


unsigned long lastMillis = 0;
const long interval = 500;
String selectedProfile = "mountain";











// ===== Enhanced Environmental Analysis =====
// Improved data structures
struct EnvData {
  float temp;
  float humidity;
  float pressure;
  float altitude;
  int aqi;
  float dew_point;
  String direction;
  float heading;
  bool is_valid = true;
};

struct EnvProfile {
  String name;
  float temp;
  float humidity;
  float pressure;
  float altitude;
  int aqi;
  float temp_tolerance;
  float humidity_tolerance;
  String description;
};

struct DeviationAnalysis {
  float value;
  float ideal;
  float abs_dev;
  float rel_dev;
  String status;
  float score;
};

// Global profile storage
std::vector<EnvProfile> envProfiles;

// Helper functions
float calculateDewPoint(float t, float h) {
  // Magnus formula
  float a = 17.62, b = 243.12;
  float alpha = log(h / 100.0) + (a * t) / (b + t);
  return (b * alpha) / (a - alpha);
}

float calculateHeatIndex(float t, float h) {
  // Simplified Rothfusz regression
  if (t < 27) return t;
  return 0.5 * (t + 61.0 + ((t - 68.0) * 1.2) + (h * 0.094));
}

DeviationAnalysis analyzeParameter(float value, float ideal, float tolerance) {
  DeviationAnalysis da;
  da.value = value;
  da.ideal = ideal;
  da.abs_dev = value - ideal;
  da.rel_dev = (ideal != 0) ? 100.0 * da.abs_dev / ideal : 0;

  float tolerance_range = tolerance;
  if (fabs(da.abs_dev) <= tolerance_range) {
    da.status = "Optimal";
    da.score = 1.0;
  } else if (value > ideal) {
    da.status = "High";
    da.score = fmax(0.0, 1.0 - fabs(da.abs_dev) / (tolerance_range * 3));
  } else {
    da.status = "Low";
    da.score = fmax(0.0, 1.0 - fabs(da.abs_dev) / (tolerance_range * 3));
  }

  return da;
}

// Initialize profiles in setup()
void initEnvironmentalProfiles() {
  envProfiles = {
    { "mountain", 10.0, 50.0, 850.0, 2000.0, 40, 5.0, 15.0, "High altitude environment" },
    { "cave", 15.0, 90.0, 1012.0, 0.0, 20, 3.0, 10.0, "Underground with high humidity" },
    { "desert", 40.0, 20.0, 1005.0, 500.0, 60, 8.0, 30.0, "Arid and hot conditions" },
    { "forest", 25.0, 70.0, 1000.0, 400.0, 35, 4.0, 20.0, "Wooded area with moderate climate" },
    { "urban", 30.0, 50.0, 1013.0, 200.0, 100, 6.0, 25.0, "City environment with pollution" },
    { "rainy", 22.0, 85.0, 1008.0, 100.0, 60, 3.0, 15.0, "Wet weather conditions" }
  };
}

// Enhanced sensor reading
EnvData readSensors() {
  EnvData data;

  // Basic validation
  if (isnan(sData.Temp) || isnan(sData.Humid) || isnan(sData.Pressure)) {
    data.is_valid = false;
    return data;
  }

  data.temp = sData.Temp;
  data.humidity = constrain(sData.Humid, 0, 100) - 15;
  data.pressure = constrain(sData.Pressure, 300, 1100);
  data.altitude = sData.Alt;
  data.aqi = 125;  // Still hardcoded

  // Calculate dew point
  data.dew_point = calculateDewPoint(data.temp, data.humidity);

  // Enhanced direction calculation
  float heading = sData.X;
  if (heading < 0) heading += 360;

  // Corrected calculation - makes 285° point north (0°)
  heading = fmod(heading - 285 + 360, 360);  // Now 285° becomes 0° (north)

  static const char *directions[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };
  int index = static_cast<int>((heading + 11.25) / 22.5) % 16;
  data.direction = directions[index];
  data.heading = heading;

  return data;
}

// Enhanced analysis and reporting
void analyzeEnvironment(const EnvData &current, const EnvProfile &ideal) {
  DeviationAnalysis temp = analyzeParameter(current.temp, ideal.temp, ideal.temp_tolerance);
  DeviationAnalysis hum = analyzeParameter(current.humidity, ideal.humidity, ideal.humidity_tolerance);
  DeviationAnalysis press = analyzeParameter(current.pressure, ideal.pressure, 10.0);
  DeviationAnalysis alt = analyzeParameter(current.altitude, ideal.altitude, 200.0);
  DeviationAnalysis aqi = analyzeParameter(static_cast<float>(current.aqi), static_cast<float>(ideal.aqi), 30.0);

  // Calculate overall score
  float total_score = (temp.score * 0.25) + (hum.score * 0.2) + (press.score * 0.15) + (alt.score * 0.2) + (aqi.score * 0.2);

  // Weather analysis
  String weather_status;
  if (current.dew_point >= current.temp - 1.0) {
    weather_status = "Fog/precipitation likely";
  } else if (current.humidity < 30) {
    weather_status = "Dry conditions";
  } else if (current.humidity > 80) {
    weather_status = "Humid conditions";
  } else {
    weather_status = "Stable conditions";
  }

  // Terrain analysis
  String terrain_type;
  if (current.altitude > 1500) {
    terrain_type = "Mountainous";
  } else if (current.pressure < 950 && current.altitude > 500) {
    terrain_type = "High plateau";
  } else if (fabs(current.pressure - 1013.25) < 5.0 && current.altitude < 100) {
    terrain_type = "Sea-level";
  } else {
    terrain_type = "Mixed terrain";
  }

  // Thermal comfort
  float heat_index = calculateHeatIndex(current.temp, current.humidity);
  String comfort_level;
  if (heat_index < 20) comfort_level = "Cold";
  else if (heat_index < 27) comfort_level = "Comfortable";
  else if (heat_index < 32) comfort_level = "Warm";
  else if (heat_index < 41) comfort_level = "Hot";
  else comfort_level = "Dangerous";


  // Button press toggle logic

  // Display-friendly report

  // Serial report
  // Serial.println("\n===== ENVIRONMENTAL ANALYSIS =====");
  // Serial.printf("Profile: %s | Match: %.1f%%\n", ideal.name.c_str(), total_score * 100);
  // Serial.printf("Temp: %.1f°C (ideal: %.1f) | %s\n", temp.value, temp.ideal, temp.status.c_str());
  // Serial.printf("Humidity: %.1f%% (ideal: %.1f) | %s\n", hum.value, hum.ideal, hum.status.c_str());
  // Serial.printf("Pressure: %.1fhPa | Altitude: %.1fm\n", press.value, alt.value);
  // Serial.printf("Comfort: %s (HI: %.1f°C) | Weather: %s\n", comfort_level.c_str(), heat_index, weather_status.c_str());
  // Serial.printf("Terrain: %s | Compass: %s (%.1f°)\n", terrain_type.c_str(), current.direction.c_str(), current.heading);
  // Serial.println("=================================");
  // ===== END Enhanced Environmental Analysis =====
}

















void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Internal pull-up resistor
  pinMode(Btn3, INPUT_PULLUP);        // Internal pull-up resistor
  pinMode(Btn4, INPUT_PULLUP);        // Internal pull-up resistor
  // pinMode(bpin, OUTPUT);  // Internal pull-up resistor

  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.softAP(ssid, password);










  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  display.clearDisplay();
  WiFi.setTxPower(WIFI_POWER_20_5dBm);
  // server.on("/webTestV1", HTTP_handleRoot);
  server.on("/webTestV3", testServe);
  server.on("/webTestV2", testV2Serve);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, slaveMac, 6);
  peer.channel = 1;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Peer add failed");
    return;
  }

  pinMode(swPin, INPUT_PULLUP);
  pinMode(2, OUTPUT);           // [ADDED] Built-in LED pin
  initEnvironmentalProfiles();  // Initialize environmental profiles
}







// Add these global variables
unsigned long lastButtonPress = 0;
const int debounceDelay = 200;  // ms







void loop() {
  server.handleClient();
  webSocket.loop();

  data.x = analogRead(xPin);
  data.y = analogRead(yPin);
  data.sw = digitalRead(swPin) == LOW ? 1 : 0;
  data.Btn = digitalRead(BUTTON_PIN) == LOW ? 1 : 0;
  // Serial.println(data.Btn);

  esp_now_send(slaveMac, (uint8_t *)&data, sizeof(data));
  Serve_On_Page();

  // ----- BLINK LOGIC START -----
  if (communicationSuccess) {
    if (millis() - lastBlinkTime >= 700) {
      lastBlinkTime = millis();
      ledState = !ledState;
      digitalWrite(2, ledState);
    }
  } else {
    digitalWrite(2, LOW);  // Keep LED off if no communication
  }

  communicationSuccess = false;  // Reset for next loop
  // ----- BLINK LOGIC END -----
  // Button press toggle logic
  // if (data.sw == 1 && prevBtnState == 0) {
  //   isAlgoMode = !isAlgoMode;
  // }
  // prevBtnState = data.sw;



  // Handle Serial input for profile selection
  // Handle profile switching with Btn4
  int currentBtn4State = digitalRead(Btn4);  // Make sure BUTTON_PIN is defined as your Btn4 pin

  if (currentBtn4State == LOW && prevBtn4State == HIGH && (millis() - lastProfileChange) > profileDebounceDelay && showDetailedData == true) {

    // Find current profile index
    size_t currentIndex = 0;
    for (; currentIndex < envProfiles.size(); currentIndex++) {
      if (envProfiles[currentIndex].name == selectedProfile) {
        break;
      }
    }

    // Move to next profile (wrap around if needed)
    currentIndex = (currentIndex + 1) % envProfiles.size();
    selectedProfile = envProfiles[currentIndex].name;

    // Update display and serial monitor
    Serial.printf("Switched to profile: %s\n", selectedProfile.c_str());

    // Update last change time
    lastProfileChange = millis();

    // Force display update-6
    // showDetailedData = true;
    display.clearDisplay();
    display.display();
  }

  prevBtn4State = currentBtn4State;


  // Environmental analysis
  if (millis() - lastMillis >= interval) {
    lastMillis = millis();
    EnvData live = readSensors();

    if (!live.is_valid) {
      Serial.println("Sensor error! Invalid data received");
    } else {
      // Find selected profile
      for (auto &profile : envProfiles) {
        if (profile.name == selectedProfile) {
          analyzeEnvironment(live, profile);
          break;
        }
      }
    }
  }



  // Improved button handling with debouncing
  int k = digitalRead(Btn3) == LOW ? 1 : 0;

  // Button press detection with debouncing
  if (k == 1 && prevBtn2State == 0 && (millis() - lastButtonPress) > debounceDelay) {
    showDetailedData = !showDetailedData;
    lastButtonPress = millis();

    // Immediately clear display when toggling
    display.clearDisplay();
    display.display();
  }
  prevBtn2State = k;

  // Unified display handling
  // Unified display handling
  if (showDetailedData) {
    // Show environmental data
    EnvData current = readSensors();

    if (current.is_valid) {
      // Find the selected profile
      for (auto &profile : envProfiles) {
        if (profile.name == selectedProfile) {
          // Now we can use 'profile' instead of 'ideal'

          // Add these lines to calculate the needed variables:





          DeviationAnalysis temp = analyzeParameter(current.temp, profile.temp, profile.temp_tolerance);
          DeviationAnalysis hum = analyzeParameter(current.humidity, profile.humidity, profile.humidity_tolerance);
          DeviationAnalysis press = analyzeParameter(current.pressure, profile.pressure, 10.0);
          DeviationAnalysis alt = analyzeParameter(current.altitude, profile.altitude, 200.0);
          DeviationAnalysis aqi = analyzeParameter(static_cast<float>(current.aqi), static_cast<float>(profile.aqi), 30.0);

          // Calculate overall score
          float total_score = (temp.score * 0.25) + (hum.score * 0.2) + (press.score * 0.15) + (alt.score * 0.2) + (aqi.score * 0.2);


          String weather_status;
          if (current.dew_point >= current.temp - 1.0) {
            weather_status = "Fog/precipitation likely";
          } else if (current.humidity < 30) {
            weather_status = "Dry conditions";
          } else if (current.humidity > 80) {
            weather_status = "Humid conditions";
          } else {
            weather_status = "Stable conditions";
          }

          String terrain_type;
          if (current.altitude > 1500) {
            terrain_type = "Mountainous";
          } else if (current.pressure < 950 && current.altitude > 500) {
            terrain_type = "High plateau";
          } else if (fabs(current.pressure - 1013.25) < 5.0 && current.altitude < 100) {
            terrain_type = "Sea-level";
          } else {
            terrain_type = "Mixed terrain";
          }

          float heat_index = calculateHeatIndex(current.temp, current.humidity);

          String comfort_level;
          if (heat_index < 20) comfort_level = "Cold";
          else if (heat_index < 27) comfort_level = "Comfortable";
          else if (heat_index < 32) comfort_level = "Warm";
          else if (heat_index < 41) comfort_level = "Hot";
          else comfort_level = "Dangerous";

          // Rest of your existing loop code...


          // Display update
          // Optimized OLED display layout
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);

          // Line 1: Profile and match score (top left) + Temp/Humidity (right aligned)
          display.setCursor(0, 0);
          display.printf("Terr: %s: %.1f%%", profile.name.substring(0, 8).c_str(), total_score * 100);  // Shorten profile name
          // display.setCursor(80, 0);
          // display.printf(" H:%d%%", static_cast<int>(current.humidity));

          // Line 2: Temp comparison
          display.setCursor(0, 11);
          display.printf("T:%.1fC/%.1f %s", current.temp, profile.temp, temp.status.substring(0, 8).c_str());

          // Line 3: Humidity comparison
          display.setCursor(0, 22);
          display.printf("H:%.1f/%.1f %s", current.humidity, profile.humidity, hum.status.substring(0, 8).c_str());

          // Line 4: Pressure and comfort
          display.setCursor(0, 33);
          display.printf("P:%.0fhPa A:%.0f m", current.pressure, current.altitude);

          // Line 5: Altitude and weather
          display.setCursor(0, 44);
          display.printf("%s & %s", weather_status.substring(0, 6).c_str(), comfort_level.substring(0, 11).c_str());

          // Line 6: Wind direction and heat index
          display.setCursor(0, 55);
          display.printf("[%s] %.0fdeg HI:%.1fC", current.direction.c_str(), current.heading, heat_index);

          display.display();
          break;
        }
      }
    }
  } else {

    display.clearDisplay();

    // --- Mode Display ---
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Mode:");

    display.setTextSize(2);
    display.setCursor(35, 0);
    if (sData.mode != 1) {
      display.print("ALGO");
    } else {
      display.print("M_C");
    }


    // --- Battery Voltage (Larger) ---
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("BV:");
    display.setTextSize(2);

    display.setCursor(25, 20);
    display.print(sData.Voltage);
    display.print("V");

    //RadarDist
    display.setTextSize(1);
    display.setCursor(109, 23);
    display.print(sData.RadarDist);
    //targetYaw
    display.setTextSize(1);
    display.setCursor(94, 38);
    display.print(sData.targetYaw);
    display.setCursor(116, 38);

    display.print("d");



    // --- Yaw ---
    display.setTextSize(1);
    display.setCursor(10, 40);
    display.print("Yaw:");
    display.setCursor(40, 40);
    display.print(sData.X);

    display.setTextSize(1);
    display.setCursor(0, 54);
    display.print("Jx:");
    display.print(data.x, 1);
    display.print(" ");

    // display.setCursor(40, 50);
    display.print("Jy:");

    display.print(data.y, 1);


    // --- FL Box on Top Right ---
    int boxWidth = 30;
    int boxHeight = 16;
    int boxX = 128 - boxWidth;  // Right-aligned
    int boxY = 0;

    // Draw white filled box
    if (sData.mode != 1) {

      display.fillRect(boxX, boxY, boxWidth, boxHeight, SSD1306_WHITE);

      // Set text color to black (to be visible on white background)
      display.setTextColor(SSD1306_BLACK);
      display.setTextSize(2);
      display.setCursor(boxX + 2, boxY + 1);  // Padding inside the box

      switch (sData.ALGOCMD) {
        case 1:
          // goAhead(225, 225);  // Forward
          // Serial.println("🚗 Moving FORWARD");
          display.print("F");
          break;
        case 2:
          // goBack(225, 225);  // Backward
          // Serial.println("🔙 Moving BACKWARD");
          display.print("B");

          break;
        case 3:
          // goLeft(150, 150);  // Turn left
          // Serial.println("↩ Turning LEFT");
          display.print("L");

          break;
        case 4:
          // goRight(150, 150);  // Turn right
          // Serial.println("↪ Turning RIGHT");
          display.print("R");

          break;
        case 5:
          // stopRobot();  // Stop
          // Serial.println("🛑 Stopping");
          display.print("S");

          break;
        default:
          // Serial.println("⚠️ Unknown command");
          // stopRobot();  // Safe fallback
          display.print("S");

          break;
      }
    } else {
      display.fillRect(boxX, boxY, boxWidth, boxHeight, SSD1306_WHITE);

      // Set text color to black (to be visible on white background)
      display.setTextColor(SSD1306_BLACK);
      display.setTextSize(2);
      display.setCursor(boxX + 2, boxY + 1);  // Padding inside the box
      if (data.x > 1900 && data.y < 1300) {
        display.print("BR");
      } else if (data.x > 1900 && data.y > 1900) {
        display.print("BL");
      } else if (data.x < 1300 && data.y < 1300) {
        display.print("FR");
      } else if (data.x < 1300 && data.y > 1900) {
        display.print("FL");
      } else if (data.x < 1300) {
        display.print(" F");
      } else if (data.x > 1900) {
        display.print(" B");
      } else if (data.y > 1900) {
        display.print(" L");
      } else if (data.y < 1300) {
        display.print(" R");
      } else {
        display.print(" S");
      }
    }

    // Reset text color back to white for further display use
    display.setTextColor(SSD1306_WHITE);


    display.display();
  }
  delay(5);  // Performance tuning
}
//Tried and tested code By- Naitik Srivastava
