import os from "os";
import cors from "cors";
import express from "express";
import { MapData } from "./types";
import {
  MOCK_MAP_EXPLORED,
  MOCK_MAP_MARKERS,
  MOCK_MAP_PERIMETER,
  MOCK_MAP_WAYPOINTS,
} from "./utils/constants";

const port = 5000;
const app = express();
const networkInterfaces = os.networkInterfaces();

let searchPerimeter: Array<MapData> = MOCK_MAP_PERIMETER;

let droneInterestMarker: Array<MapData> = MOCK_MAP_MARKERS;
let droneTargetWaypoints: Array<MapData> = MOCK_MAP_WAYPOINTS;
let droneExploredPositions: Array<MapData> = MOCK_MAP_EXPLORED;

let splotInterestMarker: Array<MapData> = MOCK_MAP_MARKERS;
let splotTargetWaypoints: Array<MapData> = MOCK_MAP_WAYPOINTS;
let splotExploredPositions: Array<MapData> = MOCK_MAP_EXPLORED;

app.use(cors());
app.use(express.json({ limit: "2mb" }));
app.use(express.urlencoded({ extended: true }));

app.get("/", (req, res) => {
  res.send("Splot API");
});

app.get("/search/perimeter", (req, res) => {
  console.log("GET /search/perimeter");
  res.send(searchPerimeter);
});

app.post("/search/perimeter", (req, res) => {
  searchPerimeter = req.body;
  console.log("POST /search/perimeter");
  res.json(searchPerimeter);
});

app.get("/drone/interest", (req, res) => {
  console.log("GET /drone/interest");
  res.send(droneInterestMarker);
});

app.post("/drone/interest", (req, res) => {
  droneInterestMarker = req.body;
  console.log("POST /drone/interest");
  res.json(droneInterestMarker);
});

app.get("/drone/waypoints", (req, res) => {
  console.log("GET /drone/waypoints");
  res.send(droneTargetWaypoints);
});

app.post("/drone/waypoints", (req, res) => {
  droneTargetWaypoints = req.body;
  console.log("POST /drone/waypoints");
  res.json(droneTargetWaypoints);
});

app.get("/drone/explored", (req, res) => {
  console.log("GET /drone/explored");
  res.send(droneExploredPositions);
});

app.post("/drone/explored", (req, res) => {
  droneExploredPositions = req.body;
  console.log("POST /drone/explored");
  res.json(droneExploredPositions);
});

app.get("/splot/interest", (req, res) => {
  console.log("GET /splot/interest");
  res.send(splotInterestMarker);
});

app.post("/splot/interest", (req, res) => {
  splotInterestMarker = req.body;
  console.log("POST /splot/interest");
  res.json(splotInterestMarker);
});

app.get("/splot/waypoints", (req, res) => {
  console.log("GET /splot/waypoints");
  res.send(splotTargetWaypoints);
});

app.post("/splot/waypoints", (req, res) => {
  splotTargetWaypoints = req.body;
  console.log("POST /splot/waypoints");
  res.json(splotTargetWaypoints);
});

app.get("/splot/explored", (req, res) => {
  console.log("GET /splot/explored");
  res.send(splotExploredPositions);
});

app.post("/splot/explored", (req, res) => {
  splotExploredPositions = req.body;
  console.log("POST /splot/explored");
  res.json(splotExploredPositions);
});

app.listen(port, () => {
  console.log(`Server: http://localhost:${port}`);
  for (const key in networkInterfaces) {
    const networkInterface = networkInterfaces[key];
    for (const network of networkInterface as any) {
      if (network.family === "IPv4" && !network.internal) {
        console.log(`Network: http://${network.address}:${port}`);
      }
    }
  }
});
