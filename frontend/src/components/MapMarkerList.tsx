import { MapMarker } from '../types';
import { Trash2 } from 'lucide-react';
import MapMarkerInput from './MapMarkerInput';

type MapMarkerListProps = {
  markers: Array<MapMarker>;
  setMarkers: (markers: Array<MapMarker>) => void;
};

export default function MapMarkerList({
  markers,
  setMarkers,
}: MapMarkerListProps) {
  const handleDelete = (id: string) => {
    const result = markers.filter(
      (coordinate) => coordinate.id !== id,
    );
    setMarkers(result);
  };

  const handleNew = (newItem: MapMarker) => {
    setMarkers([...markers, newItem]);
  }

  return (
    <div>
      <h2>Map Marker List</h2>
      <MapMarkerInput handleNew={handleNew} />
      <ol>
        {markers.map((coordinate: MapMarker) => (
          <li
            key={`list-marker-${coordinate.id}`}
            testdata-id={`list-marker-${coordinate.id}`}
          >
            <div className="flex-row padding-10">
              <h4>{coordinate.id}</h4>
              <p>
                {coordinate.coords.lat},{coordinate.coords.lng}
              </p>
              <p>{coordinate.desc}</p>
              <button id="delete" onClick={() => handleDelete(coordinate.id)}>
                <Trash2 />
              </button>
            </div>
          </li>
        ))}
      </ol>
    </div>
  );
}
