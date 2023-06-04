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

let image;
let imageReady = false;
let originalImageDimensions = null;
let scaledImageDimensions = null;


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
  console.log('MAP: loading image');
  image = new Image();
  image.src = 'room_map.png';
  image.onload = () => {
    console.log('MAP: image loaded')
    imageReady = true;
    originalImageDimensions = {width: image.width, height: image.height};
    console.log(originalImageDimensions);
    scaledImageDimensions = calculateAspectRatioFit(originalImageDimensions.width, originalImageDimensions.height, 600, 600);
    console.log(scaledImageDimensions);

    const canvas = document.getElementById('map-canvas');
    canvas.height = scaledImageDimensions.height;
    canvas.width = scaledImageDimensions.width;
    return drawMap();
  }
}

function calculateAspectRatioFit(srcWidth, srcHeight, maxWidth, maxHeight) {

  let ratio = Math.min(maxWidth / srcWidth, maxHeight / srcHeight);

  return { width: srcWidth*ratio, height: srcHeight*ratio };
}


// Function to draw the map (known data points and lines)
function drawMap() {
  console.log("MAP: draw function")
  if (!imageReady) {
    console.log("MAP: not ready")
    return loadImage();
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
  console.log("DOM content loaded")
  drawMap();
});
  
// export {
//   createMapTopicCallbackMapping
// };