import express, { Request, Response } from "express";
import { MapData } from "../types";

const explored = express.Router();

let droneExploredPositions: Array<MapData> = [];
let splotExploredPositions: Array<MapData> = [];

explored.get("/drone", (req, res) => {
  console.log("GET /explored/drone");
  res.send(droneExploredPositions);
});

explored.delete("/drone/:id", (req, res) => {
  console.log("DELETE /explored/drone", req.params.id);
  const newDroneExploredPositions = droneExploredPositions.filter(
    (explored) => explored.id !== req.params.id
  );
  droneExploredPositions = newDroneExploredPositions;
  res.send(droneExploredPositions);
});

explored.post("/drone", (req, res) => {
  console.log("POST /explored/drone", req.body);
  droneExploredPositions = [...droneExploredPositions, req.body];
  res.json(droneExploredPositions);
});

explored.get("/splot", (req, res) => {
  console.log("GET /explored/splot");
  res.send(splotExploredPositions);
});

explored.delete("/splot/:id", (req, res) => {
  console.log("DELETE /explored/splot", req.params.id);
  const newSplotExploredPositions = splotExploredPositions.filter(
    (explored) => explored.id !== req.params.id
  );
  splotExploredPositions = newSplotExploredPositions;
  res.send(splotExploredPositions);
});

explored.post("/splot", (req, res) => {
  console.log("POST /explore/splot", req.body);
  splotExploredPositions = [...splotExploredPositions, req.body];
  res.json(splotExploredPositions);
});

export default explored;
