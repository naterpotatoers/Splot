import { useState } from 'react';
import { MapData } from '../types';
import { Popup, Marker, useMapEvents } from 'react-leaflet';

// TODO: Pass mapData in as a prop
export default function MarkerOnDoubleClick() {
  const [testMarkers, setTestMarkers] = useState<Array<MapData>>([]);
  useMapEvents({
    dblclick(e) {
      const newMarker = {
        id: `manual-marker-${Date.now().toString()}`,
        coords: e.latlng,
        desc: `Test Marker ${testMarkers.length + 1}`,
      };
      console.log(newMarker);
      setTestMarkers((prevMarkers) => [...prevMarkers, newMarker]);
    },
  });

  return (
    <>
      {testMarkers.map((marker) => (
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
                setTestMarkers((prevMarkers) =>
                  prevMarkers.filter((m) => m.id !== marker.id),
                );
              }}
            >
              Delete
            </button>
          </Popup>
        </Marker>
      ))}
    </>
  );
}
