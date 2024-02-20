import { MapMarker } from '../types';
import { Trash2 } from 'lucide-react';

type MapMarkerListProps = {
  markers: Array<MapMarker>;
  removeMarker: (marker: MapMarker) => void;
};

export default function MapMarkerList({
  markers,
  removeMarker,
}: MapMarkerListProps) {
  return (
    <div>
      <h2>Map Marker List</h2>
      {markers.map((marker: MapMarker) => (
        <div key={marker.id} className="flex-row">
          <p>{marker.id}</p>
          <p>
            {marker.coords.lat},{marker.coords.lng}
          </p>
          <p>{marker.desc}</p>
          <button
            id={`remove-${marker.id}`}
            onClick={() => removeMarker(marker)}
          >
            <Trash2 />
          </button>
        </div>
      ))}
    </div>
  );
}
