// Array of known data points
const knownDataPoints = [
  {x:177,y:131},
  {x:414,y:89},
  {x:655,y:126},
  {x:944,y:93},
  {x:1151,y:170},
  {x:1442,y:139},
  {x:1599,y:184},
  {x:131,y:315},
  {x:669,y:374},
  {x:1125,y:349},
  {x:1599,y:400},
  {x:79,y:524},
  {x:271,y:612},
  {x:826,y:505},
  {x:1075,y:510},
  {x:1643,y:583},
  {x:433,y:787},
  {x:708,y:691},
  {x:1127,y:714},
  {x:1342,y:773},
  {x:1573,y:839},
  {x:315,y:970},
  {x:669,y:948},
  {x:787,y:1133},
  {x:1049,y:1206},
  {x:1245,y:1040},
  {x:1455,y:944},
  {x:79,y:1259},
  {x:229,y:1311},
  {x:781,y:1332},
  {x:1064,y:1396},
  {x:1632,y:1167},
  {x:1599,y:1320},
  {x:180,y:1553},
  {x:354,y:1626},
  {x:610,y:1588},
  {x:873,y:1626},
  {x:901,y:1497},
  {x:1091,y:1569},
  {x:1384,y:1592},
  {x:1573,y:1547}
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
let imageScaleFactors = {width: 1, height: 1};


/* Exported functions ------------------------------------------------------ */

// Callback function for current pose
export function currentPoseCallback(update) {
  currentPose = convertPointToPixels({x: update.x, y: update.y});
  // currentPose = { x: update.x, y: update.y };
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
export function plotPoint(context, x, y, color) {
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

    const canvas = document.getElementById('map-canvas');
    let scaledImageDimensions = calculateAspectRatioFit(image.width, image.height, 600, 600);
    canvas.height = scaledImageDimensions.height;
    canvas.width = scaledImageDimensions.width;

    return drawMap();
  }
}

function calculateAspectRatioFit(srcWidth, srcHeight, maxWidth, maxHeight) {

  let ratio = Math.min(maxWidth / srcWidth, maxHeight / srcHeight);
  let scaled = { width: srcWidth*ratio, height: srcHeight*ratio }
  console.log('scaling', [srcWidth, srcHeight], 'to', scaled);

  imageScaleFactors.width  = scaled.width  / srcWidth;
  imageScaleFactors.height = scaled.height / srcHeight;
  return scaled;
}

export function convertPointToPixels(point) {
  return { x: point.x *100/0.594 * imageScaleFactors.width,
          /* convert room y to image y coordinate system */
           y: (image.height - point.y *100/0.594) * imageScaleFactors.height}
}

// Function to draw the map (known data points and lines)
export function drawMap() {
  console.log("MAP: draw function")
  if (!imageReady) {
    console.log("MAP: not ready")
    return loadImage();
  }

  const xScale = imageScaleFactors.width;
  const yScale = imageScaleFactors.height;

  const canvas = document.getElementById('map-canvas');
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);

  // draw image
  context.drawImage(image, 0, 0, canvas.width, canvas.height);

  // Plot known data points
  knownDataPoints.forEach(point => {
    plotPoint(context, point.x * xScale, point.y * yScale, 'blue');
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
        startPoint.x * xScale,
        startPoint.y * yScale,
        endPoint.x * xScale,
        endPoint.y * yScale,
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