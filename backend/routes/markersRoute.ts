import express, { Request, Response } from "express";
import { MapData } from "../types";

const mapDataRoute = express.Router();

let mapData: Array<MapData> = [];

mapDataRoute.get("/", (req: Request, res: Response) => {
  console.log("GET", req.baseUrl);
  res.send(mapData);
});

mapDataRoute.put("/:id", (req: Request, res: Response) => {
  console.log("PUT", req.baseUrl, req.params.id, req.body);
  mapData = mapData.map((explored) =>
    explored.id === req.params.id ? req.body : explored
  );
  res.json(mapData);
});

mapDataRoute.post("/", (req: Request, res: Response) => {
  console.log("POST", req.baseUrl, req.body);
  mapData = [...mapData, req.body];
  res.json(mapData);
});

mapDataRoute.delete("/:id", (req: Request, res: Response) => {
  console.log("DELETE", req.baseUrl, req.params.id);
  mapData = mapData.filter((explored) => explored.id !== req.params.id);
  res.send(mapData);
});

mapDataRoute.delete("/", (req: Request, res: Response) => {
  console.log("DELETE", req.baseUrl);
  mapData = [];
  res.json(mapData);
});

export default mapDataRoute;
