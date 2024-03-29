import { Trash2 } from 'lucide-react';
import { MapData } from '../types';
import { Marker, Popup, useMapEvents } from 'react-leaflet';
import { MapReducerState } from '../reducer/mapReducer';

type MapClickProps = {
  mapData: MapReducerState;
  dispatch: any;
};

export default function MapClick({ mapData, dispatch }: MapClickProps) {
  useMapEvents({
    dblclick(e) {
      const newMapData: MapData = {
        id: `manually-set-` + Date.now().toString(),
        coords: e.latlng,
        desc: `Test Marker ${mapData.explored.length + 1}`,
      };
      console.log('mapClick', newMapData);
      // add marker
    },
  });
  return mapData.explored.map((marker) => (
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
            // remove marker
          }}
        >
          <Trash2 />
        </button>
      </Popup>
    </Marker>
  ));
}
