import {
  MapContainer,
  Marker,
  Polyline,
  Popup,
  Rectangle,
  TileLayer,
} from 'react-leaflet';
import { MapMarker } from '../types';
import { getMapCenter } from '../utils';

export default function Map({
  markers,
  perimeter,
}: {
  markers: Array<MapMarker>;
  perimeter: Array<MapMarker>;
}) {
  return (
    <MapContainer
      style={{ height: '100%', width: '100%', minHeight: '500px' }}
      center={getMapCenter(markers)}
      zoom={12}
      scrollWheelZoom={false}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      <Polyline
        pathOptions={{ color: 'black' }}
        positions={[
          [
            [36.77, -119.73],
            [36.74, -119.72],
            [36.71, -119.71],
          ],
          [
            [36.74, -119.72],
            [36.73, -119.73],
            [36.72, -119.74],
          ],
        ]}
        weight={15}
        opacity={0.5}
      >
        <Popup>Popup in Polyline</Popup>
      </Polyline>

      <Rectangle
        bounds={[
          [36.77, -119.74],
          [36.71, -119.71],
        ]}
        color="black"
        fillOpacity={0.05}
      >
        <Popup>
          <button
            onClick={() => {
              console.log('Hello');
            }}
          >
            Hello
          </button>
        </Popup>
      </Rectangle>

      {markers.map((marker) => (
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
