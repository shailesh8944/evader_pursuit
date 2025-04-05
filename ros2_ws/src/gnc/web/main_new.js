// Connect to ROS
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Default rosbridge WebSocket server URL
});

// Connection event handlers
ros.on('connection', () => {
  console.log('Connected to rosbridge WebSocket server');
});

ros.on('error', (error) => {
  console.error('Error connecting to rosbridge:', error);
});

ros.on('close', () => {
  console.log('Connection to rosbridge WebSocket server closed');
});

// Data storage
const maxDataPoints = 100;
const timeWindow = 30; // seconds to display in charts
let startTime = null;

// Add these variables at the top with other data storage variables
const scaleUpdateInterval = 5; // seconds between scale updates
let lastScaleUpdate = 0;
let fixedScales = {
  x: { min: -1, max: 1 },
  y: { min: -1, max: 1 },
  z: { min: -1, max: 1 },
  u: { min: -1, max: 1 },
  v: { min: -1, max: 1 },
  w: { min: -1, max: 1 },
  roll: { min: -10, max: 10 },
  pitch: { min: -10, max: 10 },
  yaw: { min: -180, max: 180 },
  p: { min: -1, max: 1 },
  q: { min: -1, max: 1 },
  r: { min: -1, max: 1 }
};

// Data arrays for each chart
const data = {
  timestamps: [],
  x: [],
  y: [],
  z: [],
  u: [], // surge velocity
  v: [], // sway velocity
  w: [], // heave velocity
  roll: [],
  pitch: [],
  yaw: [],
  p: [], // roll angular velocity
  q: [], // pitch angular velocity
  r: []  // yaw angular velocity
};

// Initialize all charts
let charts = {};

// Create time series chart
function createTimeSeriesChart(canvasId, label, color = 'rgb(75, 192, 192)') {
  const ctx = document.getElementById(canvasId).getContext('2d');
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: label,
        data: [],
        borderColor: color,
        backgroundColor: color.replace('rgb', 'rgba').replace(')', ', 0.1)'),
        borderWidth: 2,
        pointRadius: 0,
        tension: 0.1
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'linear',
          title: {
            display: true,
            text: 'Time (s)'
          },
          min: 0,
          max: timeWindow,
          ticks: {
            stepSize: 5
          }
        },
        y: {
          beginAtZero: false,
          grace: '10%' // Add 10% padding to prevent data from touching edges
        }
      },
      animation: {
        duration: 0
      }
    }
  });
}

// Create path chart (x vs y position)
function createPathChart() {
  const ctx = document.getElementById('pathChart').getContext('2d');
  return new Chart(ctx, {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Path',
        data: [],
        borderColor: 'rgb(54, 162, 235)',
        backgroundColor: 'rgba(54, 162, 235, 0.5)',
        pointRadius: 3,
        showLine: true,
        tension: 0.1
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'linear',
          position: 'bottom',
          title: {
            display: true,
            text: 'X Position (m)'
          },
          grace: '10%' // Add padding to prevent data from touching edges
        },
        y: {
          title: {
            display: true,
            text: 'Y Position (m)'
          },
          grace: '10%' // Add padding to prevent data from touching edges
        }
      },
      animation: {
        duration: 0
      }
    }
  });
}

// Initialize all charts
function initCharts() {
  charts.path = createPathChart();
  charts.x = createTimeSeriesChart('xPositionChart', 'X Position', 'rgb(255, 99, 132)');
  charts.y = createTimeSeriesChart('yPositionChart', 'Y Position', 'rgb(54, 162, 235)');
  charts.z = createTimeSeriesChart('zPositionChart', 'Z Position', 'rgb(75, 192, 192)');
  charts.u = createTimeSeriesChart('uVelocityChart', 'Surge Velocity', 'rgb(255, 159, 64)');
  charts.v = createTimeSeriesChart('vVelocityChart', 'Sway Velocity', 'rgb(153, 102, 255)');
  charts.w = createTimeSeriesChart('wVelocityChart', 'Heave Velocity', 'rgb(255, 205, 86)');
  charts.roll = createTimeSeriesChart('rollChart', 'Roll', 'rgb(201, 203, 207)');
  charts.pitch = createTimeSeriesChart('pitchChart', 'Pitch', 'rgb(255, 99, 132)');
  charts.yaw = createTimeSeriesChart('yawChart', 'Yaw', 'rgb(54, 162, 235)');
  charts.p = createTimeSeriesChart('pAngularVelocityChart', 'Roll Angular Velocity', 'rgb(255, 159, 64)');
  charts.q = createTimeSeriesChart('qAngularVelocityChart', 'Pitch Angular Velocity', 'rgb(153, 102, 255)');
  charts.r = createTimeSeriesChart('rAngularVelocityChart', 'Yaw Angular Velocity', 'rgb(255, 205, 86)');
}

// Convert quaternion to Euler angles (in degrees)
function quaternionToEuler(q) {
  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const sinp = 2 * (q.w * q.y - q.z * q.x);
  let pitch;
  if (Math.abs(sinp) >= 1) {
    pitch = Math.sign(sinp) * Math.PI / 2; // use 90 degrees if out of range
  } else {
    pitch = Math.asin(sinp);
  }

  // Yaw (z-axis rotation)
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  // Convert to degrees
  return {
    roll: roll * 180 / Math.PI,
    pitch: pitch * 180 / Math.PI,
    yaw: yaw * 180 / Math.PI
  };
}

// Update charts with new data
function updateCharts() {
  const currentTime = data.timestamps.length > 0 ? data.timestamps[data.timestamps.length - 1] : 0;
  
  // Only update scales periodically to prevent constant resizing
  const shouldUpdateScales = (currentTime - lastScaleUpdate) > scaleUpdateInterval;
  
  // Calculate min/max for better axis scaling
  const calculateAxisRange = (dataArray) => {
    if (dataArray.length === 0) return { min: -1, max: 1 };
    const min = Math.min(...dataArray);
    const max = Math.max(...dataArray);
    
    // Add padding and ensure min != max to avoid Chart.js warnings
    const range = max - min;
    const padding = Math.max(range * 0.2, 0.1); // At least 0.1 padding
    
    return { 
      min: min - padding, 
      max: max + padding 
    };
  };

  // Update path chart (x vs y)
  charts.path.data.datasets[0].data = data.x.map((x, i) => ({ x: x, y: data.y[i] }));
  
  // Update scales only periodically
  if (shouldUpdateScales && data.x.length > 0 && data.y.length > 0) {
    // Update fixed scales for path chart
    fixedScales.x = calculateAxisRange(data.x);
    fixedScales.y = calculateAxisRange(data.y);
    lastScaleUpdate = currentTime;
  }
  
  // Apply fixed scales to path chart
  charts.path.options.scales.x.min = fixedScales.x.min;
  charts.path.options.scales.x.max = fixedScales.x.max;
  charts.path.options.scales.y.min = fixedScales.y.min;
  charts.path.options.scales.y.max = fixedScales.y.max;
  charts.path.update();

  // Update time series charts
  const updateTimeSeriesChart = (chart, dataArray, scaleKey) => {
    chart.data.labels = data.timestamps;
    chart.data.datasets[0].data = dataArray;
    
    // Update fixed scales periodically
    if (shouldUpdateScales && dataArray.length > 0) {
      fixedScales[scaleKey] = calculateAxisRange(dataArray);
    }
    
    // Apply fixed y-axis scale
    chart.options.scales.y.min = fixedScales[scaleKey].min;
    chart.options.scales.y.max = fixedScales[scaleKey].max;
    
    // Set fixed x-axis scale based on time window
    chart.options.scales.x.min = Math.max(0, currentTime - timeWindow);
    chart.options.scales.x.max = Math.max(timeWindow, currentTime);
    
    chart.update();
  };

  updateTimeSeriesChart(charts.x, data.x, 'x');
  updateTimeSeriesChart(charts.y, data.y, 'y');
  updateTimeSeriesChart(charts.z, data.z, 'z');
  updateTimeSeriesChart(charts.u, data.u, 'u');
  updateTimeSeriesChart(charts.v, data.v, 'v');
  updateTimeSeriesChart(charts.w, data.w, 'w');
  updateTimeSeriesChart(charts.roll, data.roll, 'roll');
  updateTimeSeriesChart(charts.pitch, data.pitch, 'pitch');
  updateTimeSeriesChart(charts.yaw, data.yaw, 'yaw');
  updateTimeSeriesChart(charts.p, data.p, 'p');
  updateTimeSeriesChart(charts.q, data.q, 'q');
  updateTimeSeriesChart(charts.r, data.r, 'r');
}

// Subscribe to odometry topic
function subscribeToOdometry() {
  const odometryTopic = new ROSLIB.Topic({
    ros: ros,
    name: 'sookshma_00/odometry',
    messageType: 'nav_msgs/msg/Odometry'
  });

  odometryTopic.subscribe((message) => {
    const currentTime = new Date().getTime() / 1000;
    if (startTime === null) {
      startTime = currentTime;
    }
    const relativeTime = currentTime - startTime;

    // Extract position data
    const position = message.pose.pose.position;
    
    // Extract orientation (quaternion)
    const orientation = message.pose.pose.orientation;
    const euler = quaternionToEuler(orientation);
    
    // Extract linear velocity
    const linearVel = message.twist.twist.linear;
    
    // Extract angular velocity
    const angularVel = message.twist.twist.angular;

    // Add data to arrays
    data.timestamps.push(relativeTime);
    data.x.push(position.x);
    data.y.push(position.y);
    data.z.push(position.z);
    data.u.push(linearVel.x);
    data.v.push(linearVel.y);
    data.w.push(linearVel.z);
    data.roll.push(euler.roll);
    data.pitch.push(euler.pitch);
    data.yaw.push(euler.yaw);
    data.p.push(angularVel.x);
    data.q.push(angularVel.y);
    data.r.push(angularVel.z);

    // Trim data arrays to keep only the last maxDataPoints
    if (data.timestamps.length > maxDataPoints) {
      data.timestamps.shift();
      data.x.shift();
      data.y.shift();
      data.z.shift();
      data.u.shift();
      data.v.shift();
      data.w.shift();
      data.roll.shift();
      data.pitch.shift();
      data.yaw.shift();
      data.p.shift();
      data.q.shift();
      data.r.shift();
    }

    // Update charts with new data
    updateCharts();
  });
}

// Initialize everything when the page loads
window.onload = function() {
  initCharts();
  subscribeToOdometry();
};
