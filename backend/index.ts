import os from "os";
import cors from "cors";
import express from "express";
import { MapData } from "./types";
import {
  MOCK_MAP_MARKERS,
  MOCK_MAP_PERIMETER,
  MOCK_MAP_WAYPOINTS,
} from "./utils/constants";
import explored from "./routes/explored";

const port = 5000;
const app = express();
const networkInterfaces = os.networkInterfaces();

let searchPerimeter: Array<MapData> = MOCK_MAP_PERIMETER;

let droneInterestMarker: Array<MapData> = MOCK_MAP_MARKERS;
let droneTargetWaypoints: Array<MapData> = MOCK_MAP_WAYPOINTS;

let splotInterestMarker: Array<MapData> = MOCK_MAP_MARKERS;
let splotTargetWaypoints: Array<MapData> = MOCK_MAP_WAYPOINTS;

app.use(cors());
app.use(express.json({ limit: "2mb" }));
app.use(express.urlencoded({ extended: true }));

app.get("/", (req, res) => {
  res.send("Splot API");
});

app.use("/explored", explored);

// app.get("/search/perimeter", (req, res) => {
//   console.log("GET /search/perimeter");
//   res.send(searchPerimeter);
// });

// app.post("/search/perimeter", (req, res) => {
//   console.log("POST /search/perimeter", req.body);
//   searchPerimeter = [...searchPerimeter, req.body];
//   res.json(searchPerimeter);
// });

// app.delete("/search/perimeter/:id", (req, res) => {
//   const newSearchPerimeter = searchPerimeter.filter(
//     (perimeter) => perimeter.id !== req.params.id
//   );
//   searchPerimeter = newSearchPerimeter;
//   console.log("DELETE /search/perimeter");
//   res.send(searchPerimeter);
// });

// app.get("/drone/interest", (req, res) => {
//   console.log("GET /drone/interest");
//   res.send(droneInterestMarker);
// });

// app.post("/drone/interest", (req, res) => {
//   console.log("POST /drone/interest", req.body);
//   droneInterestMarker = [...droneInterestMarker, req.body];
//   res.json(droneInterestMarker);
// });

// app.get("/drone/waypoints", (req, res) => {
//   console.log("GET /drone/waypoints");
//   res.send(droneTargetWaypoints);
// });

// app.post("/drone/waypoints", (req, res) => {
//   console.log("POST /drone/waypoints", req.body);
//   droneTargetWaypoints = [...droneTargetWaypoints, req.body];
//   res.json(droneTargetWaypoints);
// });

// app.get("/splot/interest", (req, res) => {
//   console.log("GET /splot/interest");
//   res.send(splotInterestMarker);
// });

// app.post("/splot/interest", (req, res) => {
//   console.log("POST /splot/interest", req.body);
//   splotInterestMarker = [...splotInterestMarker, req.body];
//   res.json(splotInterestMarker);
// });

// app.get("/splot/waypoints", (req, res) => {
//   console.log("GET /splot/waypoints");
//   res.send(splotTargetWaypoints);
// });

// app.post("/splot/waypoints", (req, res) => {
//   console.log("POST /splot/waypoints", req.body);
//   splotTargetWaypoints = [...splotTargetWaypoints, req.body];
//   res.json(splotTargetWaypoints);
// });

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
