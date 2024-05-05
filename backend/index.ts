import os from "os";
import cors from "cors";
import express from "express";
import { MapData } from "./types";
import {
  MOCK_MAP_MARKERS,
  MOCK_MAP_PERIMETER,
  MOCK_MAP_WAYPOINTS,
} from "./utils/constants";
import basicMapDataRoute from "./routes/basicMapDataRoute";

const port = 5000;
const app = express();
const networkInterfaces = os.networkInterfaces();

app.use(cors());
app.use(express.json({ limit: "2mb" }));
app.use(express.urlencoded({ extended: true }));

app.get("/", (req, res) => {
  res.send("Splot API");
});

app.use("/explored", basicMapDataRoute);
app.use("/waypoints", basicMapDataRoute);
app.use("/search", basicMapDataRoute);
app.use("/interest", basicMapDataRoute);

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
