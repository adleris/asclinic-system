import { createMapTopicCallbackMapping, plotPoint } from './map.js';

// Connect to Ros Bridge
var ros = new ROSLIB.Ros({
    url : 'ws://10.41.146.230:9090' // on robot
    // url : 'ws://localhost:9090'  // in Docker
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// ----------------------------------------------------------------------------

// List all Topics
const refreshBtn = document.getElementById("refresh-topic-btn");
var checkboxes = document.querySelectorAll("#checkbox-list input[type='checkbox']");

var filteredTopicList = [];

async function getTopics() {
    let topicsClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
    });

    let request = new ROSLIB.ServiceRequest();

    let topicList = await new Promise((resolve, reject) => {
        topicsClient.callService(request, function(result) {
            console.log("Getting topics...");
            resolve(result);
        })
    })

    console.log(topicList)
    // Add topics to the dropdown menu
    addTopicToDropdown(topicList, "topic-dropdown")

    console.log("add topics to console")
    addAscToConsole(topicList)

    console.log("creating mappings")
    createMapTopicCallbackMapping(ros, topicList);

    // Add topics to checkbox list
    createInputs(topicList)
    checkboxes = [];

    // Refresh event listeners
    checkboxes = document.querySelectorAll("#checkbox-list input[type='checkbox']");
    checkboxes.forEach(checkbox => {
    checkbox.addEventListener("change", function() {
        testFunction()
        });
    });
};

async function testFunction() {
    let topicsClient = new ROSLIB.Service({
        ros : ros,
        name : '/rosapi/topics',
        serviceType : 'rosapi/Topics'
        });
    
    let request = new ROSLIB.ServiceRequest();

    let topicList = await new Promise((resolve, reject) => {
        topicsClient.callService(request, function(result) {
            console.log("Getting topics...");
            resolve(result);
        })
    })


    filteredTopicList = []
    checkboxes = document.querySelectorAll("#checkbox-list input[type='checkbox']");
    checkboxes.forEach(checkbox => {
            if (checkbox.checked == true) {
                let topicListIndex = topicList["topics"].indexOf(checkbox.value)
                filteredTopicList.push(topicList["topics"][topicListIndex])
            }
        });
}

function addTopicToDropdown(topicList, dropdownID) {
    const dropdown = document.getElementById(dropdownID);

    // Remove all existing options from the dropdown menu
    while (dropdown.firstChild) {
        dropdown.removeChild(dropdown.firstChild);
    }

    // Add each topic to the dropdown menu as an option
    for (let i = 0; i < topicList["topics"].length; i++) {
        const option = document.createElement("option");
        option.value = topicList["topics"][i];
        option.text = topicList["topics"][i];
        dropdown.add(option);
    }
}

refreshBtn.addEventListener("click", clearConsole);

// ----------------------------------------------------------------------------

// Implement console feature
const consoleElement = document.getElementById("console");
const clearConsoleBtn = document.getElementById("clear-console-btn");
const consoleColourOption = document.getElementById("colour-option");
const consoleTimeOption = document.getElementById("time-option");

// Define a function to add a new line to the console element
function addLineToConsole(heading, content, useColour) {
  // Create a new paragraph element with the specified text
  const newLine = document.createElement("div");
  // Create span elements for preamble and content
  const headingSpan = document.createElement("span");
  const contentSpan = document.createElement("span");
  headingSpan.innerText = heading;
  contentSpan.innerText = content;

  if (useColour){
      headingSpan.style.color = '#FF0000';
  }

  newLine.style     = "padding-left: 10px"

  // Append the spans to the new line element
  newLine.appendChild(headingSpan);
  newLine.appendChild(contentSpan);

  // Add the new paragraph element to the console element
  consoleElement.appendChild(newLine);
}

function clearConsole() {
  consoleElement.innerHTML = "";
}

// Add a click event listener to the clear console button
clearConsoleBtn.addEventListener("click", clearConsole);

// Add all elements to console that start with /asc
var listenerArray = []

function addAscToConsole(topicList){

    let n = 0;
    listenerArray = []

    for (let i=0; i < topicList["topics"].length; i++) {

        // if (!topicList["topics"][i].startsWith('/asc')) {
        //     continue;
        // }
        console.log("Adding: " + topicList["topics"][i])

        listenerArray[n] = new ROSLIB.Topic({
            ros : ros,
            name : topicList["topics"][i],
            messageType : topicList["types"][i]
        });
        listenerArray[n].subscribe(function(message) {
            //console.log('Received message on ' + listener.name + ': ' + message.data);
            const now = new Date();
            const showTime = consoleTimeOption.checked;
            const useColour = consoleColourOption.checked;
            let timeString;

            if (showTime){
                timeString = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }) + ' | ';
            } else {
                timeString = '';
            }

            let header = `${timeString}[${this.name}] `;
            let messageString = `${JSON.stringify(message)}`;

            if (toggleValue && filteredTopicList.includes(this.name)) {
                addLineToConsole(header, messageString, useColour);
            }

        });

        n++;
    }

}

function clearListeners() {
    listenerArray = [];
}

// ----------------------------------------------------------------------------

// Implementation for publishing messages

const sysStartBtn = document.getElementById("system-start");
sysStartBtn.addEventListener("click", function(){
    publishRosBool("/asc/system_start", true);
})

const photoTakenBtn = document.getElementById("taken-photo");
photoTakenBtn.addEventListener("click", function(){
    publishRosBool("/asc/photo_taken", true);
})
    

async function publishRosBool(rostopic, boolData){

    var message = new ROSLIB.Message({
        data: boolData
      });

    var topic = new ROSLIB.Topic({
        ros: ros,
        name: rostopic,
        messageType: 'std_msgs/Bool'
    });
    
    topic.publish(message);
}

/* <input type="checkbox" id="checkbox-1" value="Option 1">
<label for="checkbox-1">Option 1</label><br>

<input type="checkbox" id="checkbox-2" value="Option 2">
<label for="checkbox-2">Option 2</label><br>

<input type="checkbox" id="checkbox-3" value="Option 3">
<label for="checkbox-3">Option 3</label><br> */

// async function publishMessage1() {
//     // Retrieve the selected dropdown value
//     const dropdown = document.getElementById("topic-dropdown2");
//     const selectedValue = dropdown.options[dropdown.selectedIndex].value;

//     // Retrieve the input field value
//     const inputValue = document.getElementById("inputValue").value;

//     // Do something with the selected value and input value
//     // console.log(`Selected value: ${selectedValue}`);
//     // console.log(`Input value: ${inputValue}`);
//     // ... (insert your own code here)


//     console.log(inputValue)
//     console.log(typeof inputValue)
//     publishMessage(selectedValue, topicList["topics"].indexOf(selectedValue), inputValue)

// }

// //publishBtn.addEventListener("click", publishMessage);
// dropdown2.addEventListener("change", changePublishText); 

var toggleValue = false;

function toggleButton() {
    const button = document.getElementById('toggle-button');

    toggleValue = !toggleValue;

    if (toggleValue) {
    button.textContent = 'Stop';
    } else {
    button.textContent = 'Start';
    }

    console.log('Current value:', toggleValue);
}


function createInputs(topicList) {
    const checkboxList = document.getElementById("checkbox-list");
    checkboxList.innerHTML = ""

    for (let i=0; i < topicList["topics"].length; i++){
        let checkbox = document.createElement("input");
        checkbox.type = "checkbox";
        checkbox.id = topicList["topics"][i];
        checkbox.value = topicList["topics"][i];
    
        let label = document.createElement("label");
        label.setAttribute("for", topicList["topics"][i]);
        label.textContent = topicList["topics"][i];
    
        let lineBreak = document.createElement("br");
    
        let container = document.createElement("div");
        container.appendChild(checkbox);
        container.appendChild(label);
        container.appendChild(lineBreak);

        checkboxList.append(container);
    }
}

var button = document.getElementById("toggleButton");
var debug  = document.getElementById("debug");
var client = document.getElementById("client");
debug.style.display = "None"
var currentState = "Client";


button.addEventListener("click", function() {
    if (currentState === "Client") {
    currentState = "Debug"
    button.textContent = "Debug";
    debug.style.display = ""
    client.style.display = "None"
    
    } else {
    currentState = "Client"
    button.textContent = "Client";
    debug.style.display = "None"
    client.style.display = ""
    }
});


// Initialise Functions
getTopics()
