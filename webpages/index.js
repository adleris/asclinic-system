// Connect to Ros Bridge
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
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
    addTopicToDropdown(topicList, "topic-dropdown2")

    addAscToConsole(topicList)
};

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

// Define a function to add a new line to the console element
function addLineToConsole(text) {
  // Create a new paragraph element with the specified text
  const newLine = document.createElement("div");
  newLine.innerText = text;
  newLine.style     = "padding-left: 10px"

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

    for (i=0; i < topicList["topics"].length; i++) {

        if (!topicList["topics"][i].startsWith('/asc')) {
            continue;
        }
        console.log(topicList["topics"][i])

        listenerArray[n] = new ROSLIB.Topic({
            ros : ros,
            name : topicList["topics"][i],
            messageType : topicList["types"][i]
        });
        listenerArray[n].subscribe(function(message) {
        //console.log('Received message on ' + listener.name + ': ' + message.data);
            const now = new Date();
            const timeString = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
            addLineToConsole(`${timeString} | [${this.name}] ${message.data}`)
        });

        n++;
    }

}

// ----------------------------------------------------------------------------

// Implementation for publishing messages
const dropdown2 = document.getElementById("topic-dropdown2");
const publishBtn = document.getElementById("publish-btn");

async function changePublishText(){
    const dropdown = document.getElementById("topic-dropdown2");

    let selectedValue = dropdown.options[dropdown.selectedIndex].value;

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
    // I know, it's not pretty but it works.
    rosType = topicList["types"][topicList["topics"].indexOf(selectedValue)]

    console.log(rosType)
}

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

//publishBtn.addEventListener("click", publishMessage);
dropdown2.addEventListener("change", changePublishText); 



// async function publishMessage(topicName, messageType, messageData) {
//     // Create a ROSLIB.Topic object
//     const topic = new ROSLIB.Topic({
//       ros: ros,
//       name: topicName,
//       messageType: messageType
//     });
  
//     // Create a message object and set its data
//     const message = new ROSLIB.Message({
//       data: messageData
//     });
  
//     // Publish the message
//     topic.publish(message);
    
//     console.log(`Published message to topic '${topicName}': ${messageData}`);
//   }


// Initialise Functions
getTopics()

