import os from "os";
import cors from "cors";
import mapDataRoute from "./routes/mapDataRoute";
import express, { Request, Response } from "express";

const port = 5000;
const app = express();
const networkInterfaces = os.networkInterfaces();

app.use(cors());
app.use(express.json({ limit: "2mb" }));
app.use(express.urlencoded({ extended: true }));

app.get("/", (req: Request, res: Response) => {
  console.log("GET", req.baseUrl);
  res.send("Splot API");
});

app.use("/perimeter", mapDataRoute);

app.use("/splot/markers", mapDataRoute);
app.use("/splot/explored", mapDataRoute);
app.use("/splot/interests", mapDataRoute);
app.use("/splot/waypoints", mapDataRoute);

app.use("/scout/explored", mapDataRoute);
app.use("/scout/waypoints", mapDataRoute);

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
