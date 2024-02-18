import { MapContainer, Marker, Popup, TileLayer } from "react-leaflet";
import { MapMarker } from "../types";
import { getMapCenter } from "../utils";

export default function Map({ markers }: { markers: Array<MapMarker> }) {
  return (
    <MapContainer
      style={{ height: "100%", width: "100%", minHeight: "500px" }}
      center={getMapCenter(markers)}
      zoom={12}
      scrollWheelZoom={false}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      {markers.map((marker) => (
        <Marker key={marker.id} position={marker.coords}>
          <Popup>{marker.desc}</Popup>
        </Marker>
      ))}
    </MapContainer>
  );
}
