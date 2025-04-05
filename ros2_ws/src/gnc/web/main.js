// Connect to rosbridge websocket (adjust the URL as needed)
var ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', function () {
  console.log('Connected to rosbridge.');
});
ros.on('error', function (error) {
  console.error('Error connecting to rosbridge: ', error);
});

// Create subscribers for two odometry topics
var odomTopic1 = new ROSLIB.Topic({
  ros: ros,
  name: '/sookshma_00/nav/odometry',
  messageType: 'nav_msgs/Odometry'
});

// var odomTopic2 = new ROSLIB.Topic({
//   ros: ros,
//   name: '/sookshma_00/nav/odometry', // Adjust this topic name as needed
//   messageType: 'nav_msgs/Odometry'
// });

var actuatorTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/sookshma_00/actuator_cmd',
  messageType: 'interfaces/Actuator'
});

// Helper: convert quaternion to Euler angles
function quaternionToEuler(q) {
  var x = q.x, y = q.y, z = q.z, w = q.w;
  var sinr_cosp = 2 * (w * x + y * z);
  var cosr_cosp = 1 - 2 * (x * x + y * y);
  var roll = Math.atan2(sinr_cosp, cosr_cosp) * 180 / Math.PI;

  var sinp = 2 * (w * y - z * x);
  var pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp) * 180 / Math.PI;

  var siny_cosp = 2 * (w * z + x * y);
  var cosy_cosp = 1 - 2 * (y * y + z * z);
  var yaw = Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI;
  
  return { roll: roll, pitch: pitch, yaw: yaw };
}

// Get canvas contexts for individual charts
var pathCtx = document.getElementById('pathChart').getContext('2d');
var xPositionCtx = document.getElementById('xPositionChart').getContext('2d');
var yPositionCtx = document.getElementById('yPositionChart').getContext('2d');
var zPositionCtx = document.getElementById('zPositionChart').getContext('2d');
var uVelocityCtx = document.getElementById('uVelocityChart').getContext('2d');
var vVelocityCtx = document.getElementById('vVelocityChart').getContext('2d');
var wVelocityCtx = document.getElementById('wVelocityChart').getContext('2d');
var rollCtx = document.getElementById('rollChart').getContext('2d');
var pitchCtx = document.getElementById('pitchChart').getContext('2d');
var yawCtx = document.getElementById('yawChart').getContext('2d');
var pAngularVelocityCtx = document.getElementById('pAngularVelocityChart').getContext('2d');
var qAngularVelocityCtx = document.getElementById('qAngularVelocityChart').getContext('2d');
var rAngularVelocityCtx = document.getElementById('rAngularVelocityChart').getContext('2d');

// Create charts for each data type
var pathChart = new Chart(pathCtx, {
  type: 'scatter',
  data: {
    datasets: [
      {
        label: 'Path 1',
        data: [],
        borderColor: 'red',
        backgroundColor: 'red',
        showLine: true,
        fill: false
      },
      // {
      //   label: 'Path 2',
      //   data: [],
      //   borderColor: 'blue',
      //   backgroundColor: 'blue',
      //   showLine: true,
      //   fill: false
      // }
    ]
  },
  options: {
    animation: false,
    responsive: true,
    aspectRatio: 1,
    scales: {
      x: { display: true, title: { display: true, text: 'X Position' } },
      // y: { display: true, title: { display: true, text: 'Y Position' } }
    }
  }
});

// Create individual charts for each variable
function createSingleVariableChart(ctx, label, color1, color2) {
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: label + ' 1', data: [], borderColor: color1, fill: false },
        // { label: label + ' 2', data: [], borderColor: color2, fill: false }
      ]
    },
    options: {
      animation: false,
      responsive: true,
      scales: { x: { display: false } }
    }
  });
}

// Create all individual charts
var xPositionChart = createSingleVariableChart(xPositionCtx, 'X Position', 'red', 'blue');
var yPositionChart = createSingleVariableChart(yPositionCtx, 'Y Position', 'red', 'blue');
var zPositionChart = createSingleVariableChart(zPositionCtx, 'Z Position', 'red', 'blue');
var uVelocityChart = createSingleVariableChart(uVelocityCtx, 'U Velocity', 'red', 'blue');
var vVelocityChart = createSingleVariableChart(vVelocityCtx, 'V Velocity', 'red', 'blue');
var wVelocityChart = createSingleVariableChart(wVelocityCtx, 'W Velocity', 'red', 'blue');
var rollChart = createSingleVariableChart(rollCtx, 'Roll', 'red', 'blue');
var pitchChart = createSingleVariableChart(pitchCtx, 'Pitch', 'red', 'blue');
var yawChart = createSingleVariableChart(yawCtx, 'Yaw', 'red', 'blue');
var pAngularVelocityChart = createSingleVariableChart(pAngularVelocityCtx, 'P Angular Velocity', 'red', 'blue');
var qAngularVelocityChart = createSingleVariableChart(qAngularVelocityCtx, 'Q Angular Velocity', 'red', 'blue');
var rAngularVelocityChart = createSingleVariableChart(rAngularVelocityCtx, 'R Angular Velocity', 'red', 'blue');

// Create actuator charts container
var actuatorCharts = {};

function createActuatorChart(actuatorName) {
  var canvas = document.createElement('canvas');
  canvas.id = actuatorName + 'Chart';
  document.getElementById('actuatorCharts').appendChild(canvas);
  
  var ctx = canvas.getContext('2d');
  actuatorCharts[actuatorName] = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: actuatorName, data: [], borderColor: 'red', fill: false }
      ]
    },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      scales: { 
        x: { display: false },
        y: {
          beginAtZero: true,
          ticks: {
            maxTicksLimit: 5
          }
        }
      },
      plugins: {
        legend: {
          display: true,
          position: 'top'
        }
      }
    }
  });
}

function updateActuatorChart(actuatorName, value) {
  if (!actuatorCharts[actuatorName]) {
    createActuatorChart(actuatorName);
  }
  
  var chart = actuatorCharts[actuatorName];
  chart.data.labels.push(sampleIndex);
  chart.data.datasets[0].data.push(value);
  
  if (chart.data.labels.length > 100) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds => ds.data.shift());
  }
  chart.update();
}

var sampleIndex = 0;

function updatePathChart(x1, y1) {
  pathChart.data.datasets[0].data.push({x: x1, y: y1});
  // pathChart.data.datasets[1].data.push({x: x2, y: y2});
  if (pathChart.data.datasets[0].data.length > 100) {
    pathChart.data.datasets[0].data.shift();
    // pathChart.data.datasets[1].data.shift();
  }
  pathChart.update();
}

function updateSingleVariableChart(chart, value1) {
  chart.data.labels.push(sampleIndex);
  chart.data.datasets[0].data.push(value1);
  // chart.data.datasets[1].data.push(value2);
  if (chart.data.labels.length > 100) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds => ds.data.shift());
  }
  chart.update();
}

// Subscribe to both odometry topics
var lastMessage1 = null;

odomTopic1.subscribe(function (message) {
  lastMessage1 = message;
  processOdomMessages(lastMessage1);
});

// odomTopic2.subscribe(function (message) {
//   lastMessage2 = message;
//   if (lastMessage1) {
//     processOdomMessages(lastMessage1, lastMessage2);
//   }
// });

function processOdomMessages(message1) {
  sampleIndex++;
  
  // Extract data from first message
  var x1 = message1.pose.pose.position.x;
  var y1 = message1.pose.pose.position.y;
  var z1 = message1.pose.pose.position.z;
  var u1 = message1.twist.twist.linear.x;
  var v1 = message1.twist.twist.linear.y;
  var w1 = message1.twist.twist.linear.z;
  var p1 = message1.twist.twist.angular.x;
  var q1 = message1.twist.twist.angular.y;
  var r1 = message1.twist.twist.angular.z;
  var euler1 = quaternionToEuler(message1.pose.pose.orientation);
  
  // Extract data from second message
  // var x2 = message2.pose.pose.position.x;
  // var y2 = message2.pose.pose.position.y;
  // var z2 = message2.pose.pose.position.z;
  // var u2 = message2.twist.twist.linear.x;
  // var v2 = message2.twist.twist.linear.y;
  // var w2 = message2.twist.twist.linear.z;
  // var p2 = message2.twist.twist.angular.x;
  // var q2 = message2.twist.twist.angular.y;
  // var r2 = message2.twist.twist.angular.z;
  // var euler2 = quaternionToEuler(message2.pose.pose.orientation);
  
  // Update all charts
  updatePathChart(x1, y1);
  updateSingleVariableChart(xPositionChart, x1);
  updateSingleVariableChart(yPositionChart, y1);
  updateSingleVariableChart(zPositionChart, z1);
  updateSingleVariableChart(uVelocityChart, u1);
  updateSingleVariableChart(vVelocityChart, v1);
  updateSingleVariableChart(wVelocityChart, w1);
  updateSingleVariableChart(rollChart, euler1.roll);
  updateSingleVariableChart(pitchChart, euler1.pitch);
  updateSingleVariableChart(yawChart, euler1.yaw);
  updateSingleVariableChart(pAngularVelocityChart, p1);
  updateSingleVariableChart(qAngularVelocityChart, q1);
  updateSingleVariableChart(rAngularVelocityChart, r1);
}

actuatorTopic.subscribe(function (message) {
  for (var i = 0; i < message.actuator_names.length; i++) {
    updateActuatorChart(message.actuator_names[i], message.actuator_values[i]);
  }
});