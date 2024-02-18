import { MapContainer, Marker, Popup, TileLayer } from "react-leaflet";
import { MapCoordinate } from "../types";

export default function Map({ lat, lng }: MapCoordinate) {
  // const center = getMapCenter();
  return (
    <MapContainer
      style={{ height: "100%", width: "100%", minHeight: "500px" }}
      center={{ lat, lng }}
      zoom={12}
      scrollWheelZoom={false}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <Marker position={{lat, lng}}>
        <Popup>Last known location</Popup>
      </Marker>
    </MapContainer>
  );
}
