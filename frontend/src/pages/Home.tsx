import { useState } from "react";
import { MAP_MARKERS } from "../constants";

import Map from "../components/Map";
import MapMarkerList from "../components/MapMarkerList";
import MapMarkerInput from "../components/MapMarkerInput";
import { MapMarker } from "../types";

export default function Home() {
  const [markers, setMarkers] = useState<Array<MapMarker>>(MAP_MARKERS);
  return (
    <div>
      <h1>Splot</h1>
      <Map markers={markers} />
      <MapMarkerInput setMarkers={setMarkers} />
      <MapMarkerList markers={markers} setMarkers={setMarkers} />
    </div>
  );
}
