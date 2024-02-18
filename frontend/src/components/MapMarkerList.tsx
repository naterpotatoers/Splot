import { MapMarker } from "../types";
import { Trash2 } from "lucide-react";

type MapMarkerListProps = {
  markers: Array<MapMarker>;
  setMarkers: (markers: Array<MapMarker>) => void;
};

export default function MapMarkerList({ markers, setMarkers }: MapMarkerListProps) {

  const handleDelete = (coordinateId: string) => {
    const newMarkers = markers.filter((coordinate) => coordinate.id !== coordinateId);
    setMarkers(newMarkers);
  };

  return (
    <div>
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
