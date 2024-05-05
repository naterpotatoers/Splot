import os from "os";
import cors from "cors";
import markerRoute from "./routes/markersRoute";
import exploredRoute from "./routes/exploredRoute";
import perimeterRoute from "./routes/perimeterRoute";
import splotWaypointRoute from "./routes/splotWaypointsRoute";
import scoutWaypointRoute from "./routes/scoutWaypointsRoute";
import express, { Request, Response } from "express";

const port = 8000;
const app = express();
const networkInterfaces = os.networkInterfaces();

app.use(cors());
app.use(express.json({ limit: "2mb" }));
app.use(express.urlencoded({ extended: true }));

app.get("/", (req: Request, res: Response) => {
  console.log("GET", req.baseUrl);
  res.send("Splot API");
});

app.use("/markers", markerRoute);
app.use("/explored", exploredRoute);
app.use("/perimeter", perimeterRoute);
app.use("/splot/waypoints", splotWaypointRoute);
app.use("/scout/waypoints", scoutWaypointRoute);

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
