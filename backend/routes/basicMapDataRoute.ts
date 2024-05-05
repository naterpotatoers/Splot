import express, { Request, Response } from "express";
import { MapData } from "../types";

const basicMapDataRoute = express.Router();

let droneMapData: Array<MapData> = [];
let splotMapData: Array<MapData> = [];

basicMapDataRoute.get("/drone", (req, res) => {
  console.log("GET /explored/drone");
  res.send(droneMapData);
});

basicMapDataRoute.delete("/drone/:id", (req, res) => {
  console.log("DELETE /explored/drone", req.params.id);
  droneMapData = droneMapData.filter(
    (explored) => explored.id !== req.params.id
  );
  res.send(droneMapData);
});

basicMapDataRoute.post("/drone", (req, res) => {
  console.log("POST /explored/drone", req.body);
  droneMapData = [...droneMapData, req.body];
  res.json(droneMapData);
});

basicMapDataRoute.get("/splot", (req, res) => {
  console.log("GET /explored/splot");
  res.send(splotMapData);
});

basicMapDataRoute.delete("/splot/:id", (req, res) => {
  console.log("DELETE /explored/splot", req.params.id);
  splotMapData = splotMapData.filter(
    (explored) => explored.id !== req.params.id
  );
  res.send(splotMapData);
});

basicMapDataRoute.post("/splot", (req, res) => {
  console.log("POST /explore/splot", req.body);
  splotMapData = [...splotMapData, req.body];
  res.json(splotMapData);
});

export default basicMapDataRoute;
