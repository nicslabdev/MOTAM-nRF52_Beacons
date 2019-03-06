/********************************************************
* NodeJS server that reads the road side unit IMEI sent *
* by NB-IoT and responds with some JSON frame           *
* MOTAM project: https://www.nics.uma.es/projects/motam *
* Created by Manuel Montenegro, Mar 06, 2019.    V. 0.1 *
********************************************************/

const https = require("https"), express=require("express"),
  fs = require("fs");

const options = { 
    key: fs.readFileSync('certs/server.key'), 
    cert: fs.readFileSync('certs/server.crt'), 
    ca: fs.readFileSync('certs/ca.crt'), 
    requestCert: true, 
    rejectUnauthorized: true
}; 



function getResponseByImei (imei) {

  let trafficLightConfJson = {"type":"trafficLightConf","specificData":{"redTime":30,"yellowTime":5,"greenTime":15}};
  let trafficLightEmergencyJson = {"type":"trafficLightEmergency","specificData":{"trafficLightState":2}};
  let infoPanelJson = {"type":"infoPanel","specificData":{"info":"Trabajos de poda en arcen izquierdo"}};

  if (imei == "34121343Z") {
    if (Math.random()<0.5) {
      return JSON.stringify(trafficLightConfJson);
    } else {
      return JSON.stringify(trafficLightEmergencyJson);
    }

  } else if (imei == "24364973T") {
    return JSON.stringify(infoPanelJson);
  }
}


const app = express();

app.use((req, res) => {
  res.writeHead(200);
  res.end( getResponseByImei (req.socket.getPeerCertificate().subject.CN) );
});

https.createServer(options, app).listen(8080);
