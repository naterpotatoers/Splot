import {
  MapContainer,
  Marker,
  Polyline,
  Popup,
  Rectangle,
  TileLayer,
} from 'react-leaflet';
import { MapData } from '../types';
import { getMapCenter } from '../utils';

export default function Map({ mapData }: { mapData: MapData }) {
  const perimeterCoordinates = mapData.perimeter.map((perimeter) => perimeter.coords);
  const exploredCoordinates = mapData.explored.map((explored) => explored.coords);


  return (
    <MapContainer
      style={{ height: '100%', width: '100%', minHeight: '500px' }}
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
        weight={15}
        opacity={0.5}
      >
        <Popup>Popup in Polyline</Popup>
      </Polyline>

      <Rectangle
        bounds={perimeterCoordinates.length > 0 ? perimeterCoordinates : [[0, 0], [0, 0]]}
        color="black"
        fillOpacity={0.05}
      />

      {mapData.markers.map((marker) => (
        <Marker key={marker.id} position={marker.coords}>
          <Popup>
            <h5>{marker.desc}</h5>
            <button
              onClick={() => {
                console.log(`Hello from ${marker.id}`);
              }}
            >
              Hello
            </button>
          </Popup>
        </Marker>
      ))}
    </MapContainer>
  );
}
