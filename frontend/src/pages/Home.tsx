import Map from "../components/Map";
import MapMarkerList from "../components/MapMarkerList";
import { MAP_MARKERS } from "../constants";

export default function Home() {
  return (
    <div>
      Home
      <Map latitude={36.7378} longitude={-119.73} />
      <MapMarkerList markers={MAP_MARKERS} />
    </div>
  );
}
