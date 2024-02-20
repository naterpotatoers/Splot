import { useReducer } from 'react';
import { DEFAULT_MAP_DATA } from '../constants';
import { MapData } from '../types';

import Map from '../components/Map';
import mapReducer from '../reducer/mapReducer';
import MapInput from '../components/MapInput';
import MapList from '../components/MapList';

export default function Home() {
  const [mapData, dispatch] = useReducer(mapReducer, DEFAULT_MAP_DATA);

  const handleAddMarker = (marker: MapData) => {
    dispatch({ type: 'marker_added', payload: marker });
  };

  const handleRemoveMarker = (marker: MapData) => {
    dispatch({ type: 'marker_removed', payload: marker });
  };

  const handleAddPerimeter = (perimeter: MapData) => {
    dispatch({ type: 'perimeter_added', payload: perimeter });
  };

  const handleRemovePerimeter = (perimeter: MapData) => {
    dispatch({ type: 'perimeter_removed', payload: perimeter });
  };

  const handleAddExplored = (explored: MapData) => {
    dispatch({ type: 'explored_added', payload: explored });
  };

  const handleRemoveExplored = (explored: MapData) => {
    dispatch({ type: 'explored_removed', payload: explored });
  };

  return (
    <div>
      <h1>Splot</h1>
      <Map mapData={mapData} />
      <div className="grid-col-autofill">
        <div id="map-marker-data">
          <MapInput label="Marker" create={handleAddMarker} />
          <MapList
            label="Marker"
            list={mapData.markers}
            remove={handleRemoveMarker}
          />
        </div>
        <div id="map-perimeter-data">
          <MapInput label="Perimeter" create={handleAddPerimeter} />
          <MapList
            label="Perimeter"
            list={mapData.perimeter}
            remove={handleRemovePerimeter}
          />
        </div>
        <div id="map-explored-data">
          <MapInput label="Explored" create={handleAddExplored} />
          <MapList
            label="Explored"
            list={mapData.explored}
            remove={handleRemoveExplored}
          />
        </div>
      </div>
    </div>
  );
}
