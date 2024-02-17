import { MapContainer, Marker, Popup, TileLayer } from "react-leaflet";
import { GpsCoordinate } from "../types";

export default function Map({ latitude, longitude }: GpsCoordinate) {
  return (
    <MapContainer
      style={{ height: "100%", width: "100%", minHeight: "500px" }}
      center={[latitude, longitude]}
      zoom={12}
      scrollWheelZoom={false}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <Marker position={[latitude, longitude]}>
        <Popup>Last known location</Popup>
      </Marker>
      <Marker position={[latitude, longitude]}>
        <Popup>Last known location</Popup>
      </Marker>
    </MapContainer>
  );
}
