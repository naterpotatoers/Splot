import { useReducer } from 'react';
import { DEFAULT_MAP_DATA } from '../constants';
import { MapData } from '../types';

import Map from '../components/Map';
import mapReducer from '../reducer/mapReducer';
import MapMarkerList from '../components/MapMarkerList';
import MapPerimeterList from '../components/MapPerimeterList';
import MapMarkerInput from '../components/MapMarkerInput';
import MapPerimeterInput from '../components/MapPerimeterInput';
import MapExploredInput from '../components/MapExploredInput';
import MapExploredList from '../components/MapExploredList';

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
          <MapMarkerInput handleNew={handleAddMarker} />
          <MapMarkerList
            markers={mapData.markers}
            removeMarker={handleRemoveMarker}
          />
        </div>
        <div id="map-perimeter-data">
          <MapPerimeterInput handleNew={handleAddPerimeter} />
          <MapPerimeterList
            perimeter={mapData.perimeter}
            removePerimeter={handleRemovePerimeter}
          />
        </div>
        <div>
          <MapExploredInput handleNew={handleAddExplored} />
          <MapExploredList
            explored={mapData.explored}
            removeExplored={handleRemoveExplored}
          />
        </div>
      </div>
    </div>
  );
}
