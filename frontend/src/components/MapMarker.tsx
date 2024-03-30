import { Trash2 } from 'lucide-react';
import { MapData } from '../types';
import { Marker, Popup } from 'react-leaflet';

type MapMarkerProps = {
  label: string;
  mapData: Array<MapData>;
  remove: (marker: MapData) => void;
};

export default function MapMarker({ label, mapData, remove }: MapMarkerProps) {
  return mapData.map((marker) => (
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
            remove(marker);
          }}
          data-testid={`${label.toLowerCase()}-delete-button`}
        >
          <Trash2 />
        </button>
      </Popup>
    </Marker>
  ));
}
