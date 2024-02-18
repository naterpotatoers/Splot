import { MapMarker } from "../types";
import { Trash2 } from "lucide-react";

export default function MapMarkerList({
  markers,
}: {
  markers: Array<MapMarker>;
}) {
  return (
    <div>
      <h2>MapMarkerList</h2>
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
              <button>
                <Trash2 />
              </button>
            </div>
          </li>
        ))}
      </ol>
    </div>
  );
}
