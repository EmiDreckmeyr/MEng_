// Get current sensor readings when the page loads
window.addEventListener('load', getReadings);
window.addEventListener('load', getRssi);
window.addEventListener('load', getNodeLocations);

function populateNodeLocations(nodeData) {
  var tableBody = document.querySelector('#node-location-table tbody');
  tableBody.innerHTML = ''; // Clear existing table rows
  
  // Extracting the array of nodes
  var nodesArray = nodeData[''];
  
  // Loop through each node
  nodesArray.forEach(function(node) {
    var row = document.createElement('tr');
    
    var nodeIdCell = document.createElement('td');
    nodeIdCell.textContent = node.id;
    row.appendChild(nodeIdCell);
    
    var longitudeCell = document.createElement('td');
    longitudeCell.textContent = node.longitude;
    row.appendChild(longitudeCell);
    
    var latitudeCell = document.createElement('td');
    latitudeCell.textContent = node.latitude;
    row.appendChild(latitudeCell);
    
    tableBody.appendChild(row);
  });
}


// Function to create a Highcharts chart
function createChart(chartId, yAxisTitle) {
  return new Highcharts.Chart({
    chart: {
      renderTo: chartId
    },
    series: [
      { name: 'Node #1', type: 'line', color: '#101D42', marker: { symbol: 'circle', radius: 3, fillColor: '#101D42' } },
      { name: 'Node #2', type: 'line', color: '#00A6A6', marker: { symbol: 'square', radius: 3, fillColor: '#00A6A6' } },
      { name: 'Node #3', type: 'line', color: '#8B2635', marker: { symbol: 'triangle', radius: 3, fillColor: '#8B2635' } },
      { name: 'Node #4', type: 'line', color: '#71B48D', marker: { symbol: 'triangle-down', radius: 3, fillColor: '#71B48D' } },
      { name: 'Node #5', type: 'line', color: '#FF1493', marker: { symbol: 'diamond', radius: 3, fillColor: '#FF1493' } }
    ],
    title: {
      text: undefined
    },
    xAxis: {
      type: 'datetime',
      dateTimeLabelFormats: { second: '%H:%M:%S' }
    },
    yAxis: {
      title: {
        text: yAxisTitle
      }
    },
    credits: {
      enabled: false
    }
  });
}

// Create Temperature Chart
var chartTemperature = createChart('chart-temperature', 'Temperature Celsius Degrees');
var chartSoilMoisture = createChart('chart-soilM', 'Soil Moisture [%]');
var chartPressure = createChart('chart-pressure', 'Pressure [hPa]');
var chartHumidity = createChart('chart-humidity', 'Humidity [%]');
var chartLux = createChart('chart-lux', 'Lux [%]');
var chartRSSI = createChart('chart-rssi', 'RSSI [dBm]');

// Plot sensor data in the corresponding chart
function plotRssiData(chart, yAxisIndex, nodeData) {
  var k = nodeData.length;
    console.log(k);
    for (var j = 0; j < k; j++) {
        var entry = nodeData[j];
        var x = (new Date(entry.date + ' ' + entry.time)).getTime();
        //var x = j;
        console.log(x);
        var y = Number(entry.rssi); // Change this to the desired data field
        console.log(y);
        // Add the data point to the chart
        //chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        if (chart.series[yAxisIndex].data.length > 60) {
          chart.series[yAxisIndex].addPoint([x, y], true, true, true);
        } else {
           chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        }
    }
}
// Plot sensor data in the corresponding chart
function plotSensorData(chart, yAxisIndex, nodeData) {
    var k = nodeData.length;
    console.log(k);
    for (var j = 0; j < k; j++) {
        var entry = nodeData[j];
        var x = (new Date(entry.date + ' ' + entry.time)).getTime();
        //var x = j;
        console.log(x);
        var y = Number(entry.temperature); // Change this to the desired data field
        console.log(y);
        // Add the data point to the chart
        //chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        if (chart.series[yAxisIndex].data.length > 60) {
          chart.series[yAxisIndex].addPoint([x, y], true, true, true);
        } else {
           chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        }
    }
}
// Plot sensor data in the corresponding chart
function plotHum(chart, yAxisIndex, nodeData) {
    var k = nodeData.length;
    console.log(k);
    for (var j = 0; j < k; j++) {
        var entry = nodeData[j];
        var x = (new Date(entry.date + ' ' + entry.time)).getTime();
        //var x = j;
        console.log(x);
        var y = Number(entry.humidity); // Change this to the desired data field
        console.log(y);
        // Add the data point to the chart
        chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        // if (chart.series[yAxisIndex].data.length > 60) {
        //     chart.series[yAxisIndex].addPoint([x, y], true, true, true);
        // } else {
        //     chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        // }
    }
}
// Plot sensor data in the corresponding chart
function plotPres(chart, yAxisIndex, nodeData) {
    var k = nodeData.length;
    console.log(k);
    for (var j = 0; j < k; j++) {
        var entry = nodeData[j];
        var x = (new Date(entry.date + ' ' + entry.time)).getTime();
        //var x = j;
        console.log(x);
        var y = Number(entry.pressure); // Change this to the desired data field
        console.log(y);
        // Add the data point to the chart
        //chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        if (chart.series[yAxisIndex].data.length > 60) {
            chart.series[yAxisIndex].addPoint([x, y], true, true, true);
        } else {
            chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        }
    }
}
// Plot sensor data in the corresponding chart
function plotLux(chart, yAxisIndex, nodeData) {
    var k = nodeData.length;
    console.log(k);
    for (var j = 0; j < k; j++) {
        var entry = nodeData[j];
        var x = (new Date(entry.date + ' ' + entry.time)).getTime();
        //var x = j;
        console.log(x);
        var y = Number(entry.lux); // Change this to the desired data field
        console.log(y);
        // Add the data point to the chart
        //chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        if (chart.series[yAxisIndex].data.length > 60) {
            chart.series[yAxisIndex].addPoint([x, y], true, true, true);
        } else {
            chart.series[yAxisIndex].addPoint([x, y], true, false, true);
        }
    }
}
// Plot sensor data in the corresponding chart
function plotSM(chart, yAxisIndex, nodeData) {
  var k = nodeData.length;
  console.log(k);
  for (var j = 0; j < k; j++) {
      var entry = nodeData[j];
      var x = (new Date(entry.date + ' ' + entry.time)).getTime();
      //var x = j;
      console.log(x);
      var y = Number(entry.soilMoisture); // Change this to the desired data field
      console.log(y);
      // Add the data point to the chart
      //chart.series[yAxisIndex].addPoint([x, y], true, false, true);
      if (chart.series[yAxisIndex].data.length > 60) {
          chart.series[yAxisIndex].addPoint([x, y], true, true, true);
      } else {
          chart.series[yAxisIndex].addPoint([x, y], true, false, true);
      }
  }
}


// Function to fetch node location data from the server
function getNodeLocations() {
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      var nodeData1 = JSON.parse(this.responseText);
      console.log(nodeData1);
      populateNodeLocations(nodeData1);
    }
  };
  xhr.open("GET", "/node-locations", true);
  xhr.send();
}

function getReadings() {
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      //var transformedData = transformData(this.responseText);
      //console.log(transformedData);
      var myObj = JSON.parse(this.responseText);
      //var myObj = transformData(this.responseText);
      console.log(myObj);

      plotSensorData(chartTemperature, 0, myObj.Node1);
      plotSensorData(chartTemperature, 1, myObj.Node2);
      plotSensorData(chartTemperature, 2, myObj.Node3);
      plotSensorData(chartTemperature, 3, myObj.Node4);
      plotSensorData(chartTemperature, 4, myObj.Node5);
      plotHum(chartHumidity, 0, myObj.Node1);
      plotHum(chartHumidity, 1, myObj.Node2);
      plotHum(chartHumidity, 2, myObj.Node3);
      plotHum(chartHumidity, 3, myObj.Node4);
      plotHum(chartHumidity, 4, myObj.Node5);
      plotPres(chartPressure, 0, myObj.Node1);
      plotPres(chartPressure, 1, myObj.Node2);
      plotPres(chartPressure, 2, myObj.Node3);
      plotPres(chartPressure, 3, myObj.Node4);
      plotPres(chartPressure, 4, myObj.Node5);
      plotLux(chartLux, 0, myObj.Node1);
      plotLux(chartLux, 1, myObj.Node2);
      plotLux(chartLux, 2, myObj.Node3);
      plotLux(chartLux, 3, myObj.Node4);
      plotLux(chartLux, 4, myObj.Node5);
      plotSM(chartSoilMoisture, 0, myObj.Node1);
      plotSM(chartSoilMoisture, 1, myObj.Node2);
      plotSM(chartSoilMoisture, 2, myObj.Node3);
      plotSM(chartSoilMoisture, 3, myObj.Node4);
      plotSM(chartSoilMoisture, 4, myObj.Node5);
    }
  };
  xhr.open("GET", "/readings", true);
  xhr.send();
}

function getRssi() {
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      var RssiObj = JSON.parse(this.responseText);
      console.log(RssiObj);

      plotRssiData(chartRSSI,0,RssiObj.Node1);
      plotRssiData(chartRSSI,1,RssiObj.Node2);
      plotRssiData(chartRSSI,2,RssiObj.Node3);
      plotRssiData(chartRSSI,3,RssiObj.Node4);
      plotRssiData(chartRSSI,4,RssiObj.Node5);
    }
  };
  xhr.open("GET", "/rssi", true);
  xhr.send();
}

// EventSource for real-time updates
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function (e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function (e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('message', function (e) {
    console.log("message", e.data);
  }, false);

  source.addEventListener('new_readings', function (e) {
    console.log("new_readings", e.data);
    var myObj = JSON.parse(e.data);
    console.log(myObj);

    // Plot data in respective charts
    plotSensorData(chartTemperature, 0, myObj.Node1);
    plotSensorData(chartTemperature, 1, myObj.Node2);
    plotSensorData(chartTemperature, 2, myObj.Node3);
    plotSensorData(chartTemperature, 3, myObj.Node4);
    plotSensorData(chartTemperature, 4, myObj.Node5);
    plotHum(chartHumidity, 0, myObj.Node1);
    plotHum(chartHumidity, 1, myObj.Node2);
    plotHum(chartHumidity, 2, myObj.Node3);
    plotHum(chartHumidity, 3, myObj.Node4);
    plotHum(chartHumidity, 4, myObj.Node5);
    plotPres(chartPressure, 0, myObj.Node1);
    plotPres(chartPressure, 1, myObj.Node2);
    plotPres(chartPressure, 2, myObj.Node3);
    plotPres(chartPressure, 3, myObj.Node4);
    plotPres(chartPressure, 4, myObj.Node5);
    plotLux(chartLux, 0, myObj.Node1);
    plotLux(chartLux, 1, myObj.Node2);
    plotLux(chartLux, 2, myObj.Node3);
    plotLux(chartLux, 3, myObj.Node4);
    plotLux(chartLux, 4, myObj.Node5);
    plotSM(chartSoilMoisture, 0, myObj.Node1);
    plotSM(chartSoilMoisture, 1, myObj.Node2);
    plotSM(chartSoilMoisture, 2, myObj.Node3);
    plotSM(chartSoilMoisture, 3, myObj.Node4);
    plotSM(chartSoilMoisture, 4, myObj.Node5);
  }, false);

  source.addEventListener('new_readings1', function (e) {
    console.log("new_readings1", e.data);
    var RssiObj = JSON.parse(this.responseText);
    console.log(RssiObj);

    plotRssiData(chartRSSI,0,RssiObj.Node1);
    plotRssiData(chartRSSI,1,RssiObj.Node2);
    plotRssiData(chartRSSI,2,RssiObj.Node3);
    plotRssiData(chartRSSI,3,RssiObj.Node4);
    plotRssiData(chartRSSI,4,RssiObj.Node5);
  }, false);
}

