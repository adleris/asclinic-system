// Array of known data points
const knownDataPoints = [
  { x: 100, y: 100 },
  { x: 200, y: 200 },
  // Add more data points here
];

// Variables to store the current pose and path coordinates
let currentPose = null;
let pathCoordinates = [];

let topicMap = {
  "/asc/control/curr_pose" : currentPoseCallback,
  "/planner/next_target" : pathCallback
};

let mapTopicListeners = [];

let imageReady = false;


/* Exported functions ------------------------------------------------------ */

// Callback function for current pose
export function currentPoseCallback(x, y) {
  currentPose = { x, y };
  drawMap();
}

// Callback function for path
export function pathCallback(coordinates) {
  pathCoordinates = coordinates;
  drawMap();
}

export function createMapTopicCallbackMapping(ros, topicList) {
  console.log("looking at topics");

  for (let i = 0; i < topicList.topics.length; i++) {
    let topic = topicList.topics[i];

    // Check if the topic exists in the topicMap
    if (topic in topicMap) {
      let callback = topicMap[topic];

      let listener = new ROSLIB.Topic({
        ros : ros,
        name : topic,
        messageType : topicList.types[i]
      })

      listener.subscribe(function(message) {
        callback(message);
      });
      
      mapTopicListeners.push(listener);
    }
  }
}

/* Internal functions ------------------------------------------------------ */

// Function to plot a single point on the canvas
function plotPoint(context, x, y, color) {
  context.beginPath();
  context.arc(x, y, 3, 0, 2 * Math.PI);
  context.fillStyle = color;
  context.fill();
}

// Function to draw a line between two points on the canvas
function drawLine(context, x1, y1, x2, y2, color) {
  context.beginPath();
  context.moveTo(x1, y1);
  context.lineTo(x2, y2);
  context.strokeStyle = color;
  context.lineWidth = 1;
  context.stroke();
}

function loadImage() {
  const image = new Image();
  image.src = 'room_map.png';
  image.onload = () => imageReady = true;
}

// Function to draw the map (known data points and lines)
function drawMap() {
  console.log("draw function")
  if (!imageReady) {
    console.log("not ready")
    loadImage();
    return;
  }

  const canvas = document.getElementById('map-canvas');
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);

  // draw image
  context.drawImage(image, 0, 0, canvas.width, canvas.height);

  // Plot known data points
  knownDataPoints.forEach(point => {
    plotPoint(context, point.x, point.y, 'blue');
  });

  // Plot current pose
  if (currentPose) {
    plotPoint(context, currentPose.x, currentPose.y, 'red');
  }

  // Draw lines between path coordinates
  if (pathCoordinates.length > 1) {
    context.strokeStyle = 'green';
    for (let i = 0; i < pathCoordinates.length - 1; i++) {
      const startPoint = pathCoordinates[i];
      const endPoint = pathCoordinates[i + 1];
      drawLine(
        context,
        startPoint.x,
        startPoint.y,
        endPoint.x,
        endPoint.y,
        'green'
      );
    }
  }
}
  
// Start plotting the known data points
document.addEventListener('DOMContentLoaded', () => {
  const canvas = document.getElementById('map-canvas');
  const context = canvas.getContext('2d');
  drawMap();
  console.log("content loaded")
});
  
// export {
//   createMapTopicCallbackMapping
// };