import { MapData } from '../types';
import {
  MapContainer,
  Marker,
  Polyline,
  Popup,
  Rectangle,
  TileLayer,
} from 'react-leaflet';
import {
  getMapCenter,
  getPerimeterCoordinates,
  groupCoordinatesById,
} from '../utils';

type MapProps = {
  perimeter: Array<MapData>;
  explored: Array<MapData>;
  markers: Array<MapData>;
};

export default function Map({ mapData }: { mapData: MapProps }) {
  const perimeterCoordinates = getPerimeterCoordinates(mapData.perimeter);
  const exploredCoordinates = groupCoordinatesById(mapData.explored);

  return (
    <MapContainer
      style={{ height: '100%', width: '100%', minHeight: '400px' }}
      center={getMapCenter(mapData.perimeter)}
      zoom={12}
      scrollWheelZoom={false}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />

      <Polyline
        pathOptions={{ color: 'black' }}
        positions={exploredCoordinates}
        weight={10}
        opacity={0.5}
      />

      <Rectangle
        bounds={perimeterCoordinates}
        color="black"
        fillOpacity={0.05}
      />

      {mapData.markers.map((marker) => (
        <Marker key={marker.id} position={marker.coords}>
          <Popup>
            <h3>{marker.id}</h3>
            <h4>Coordinates</h4>
            <p>
              {marker.coords.lat}, {marker.coords.lng}
            </p>
            <h4>Description</h4>
            <p>{marker.desc}</p>
            <button
              onClick={() => {
                console.log(`${marker.id}`);
              }}
            >
              Click
            </button>
          </Popup>
        </Marker>
      ))}
    </MapContainer>
  );
}
