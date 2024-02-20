import { MapData } from '../types';
import { Trash2 } from 'lucide-react';

type MapMarkerListProps = {
  markers: Array<MapData>;
  removeMarker: (marker: MapData) => void;
};

export default function MapMarkerList({
  markers,
  removeMarker,
}: MapMarkerListProps) {
  return (
    <>
      <h2>Map Marker List</h2>
      <div data-testid="marker-list">
        {markers.map((marker: MapData) => (
          <div key={marker.id} className="flex-row">
            <p>{marker.id}</p>
            <p>
              {marker.coords.lat},{marker.coords.lng}
            </p>
            <p>{marker.desc}</p>
            <button
              data-testid="marker-delete-button"
              onClick={() => removeMarker(marker)}
            >
              <Trash2 />
            </button>
          </div>
        ))}
      </div>
    </>
  );
}
