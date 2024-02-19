import { useState } from 'react';
import { MAP_MARKERS } from '../constants';

import Map from '../components/Map';
import MapMarkerList from '../components/MapMarkerList';
import { MapMarker } from '../types';

export default function Home() {
  const [markers, setMarkers] = useState<Array<MapMarker>>(MAP_MARKERS);
  const [perimeter, setPerimeter] = useState<Array<MapMarker>>(MAP_MARKERS);

  return (
    <div>
      <h1>Splot</h1>
      <Map markers={markers} perimeter={perimeter} />
      <div className="grid-col-2">
        <MapMarkerList markers={markers} setMarkers={setMarkers} />
        {/* <MapMarkerList markers={perimeter} setMarkers={setPerimeter} /> */}
      </div>
    </div>
  );
}
