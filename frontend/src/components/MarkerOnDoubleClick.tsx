import { Trash2 } from 'lucide-react';
import { MapData } from '../types';
import { Marker, Popup, useMapEvents } from 'react-leaflet';

type MarkerOnDoubleClickProps = {
  label: string;
  mapData: Array<MapData>;
  create: (marker: MapData) => void;
  remove: (marker: MapData) => void;
};

export default function MarkerOnDoubleClick({
  label,
  mapData,
  create,
  remove,
}: MarkerOnDoubleClickProps) {
  useMapEvents({
    dblclick(e) {
      const newMarker = {
        id: `manually-set-${label}-` + Date.now().toString(),
        coords: e.latlng,
        desc: `Test Marker ${mapData.length + 1}`,
      };
      create(newMarker);
    },
  });
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
