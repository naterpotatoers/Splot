import { useReducer } from 'react';
import { DEFAULT_MAP_DATA } from '../constants';
import { MapMarker } from '../types';

import Map from '../components/Map';
import mapReducer from '../reducer/mapReducer';
import MapMarkerList from '../components/MapMarkerList';
import MapPerimeterList from '../components/MapPerimeterList';
import MapMarkerInput from '../components/MapMarkerInput';
import MapPerimeterInput from '../components/MapPerimeterInput';

export default function Home() {
  const [mapData, dispatch] = useReducer(mapReducer, DEFAULT_MAP_DATA);

  const handleAddMarker = (marker: MapMarker) => {
    dispatch({ type: 'marker_added', payload: marker });
  };

  const handleRemoveMarker = (marker: MapMarker) => {
    dispatch({ type: 'marker_removed', payload: marker });
  };

  const handleAddPerimeter = (perimeter: MapMarker) => {
    dispatch({ type: 'perimeter_added', payload: perimeter });
  };

  const handleRemovePerimeter = (perimeter: MapMarker) => {
    dispatch({ type: 'perimeter_removed', payload: perimeter });
  };

  return (
    <div>
      <h1>Splot</h1>
      <Map mapData={mapData} />
      <div className="grid-col-2">
        <div>
          <MapMarkerInput handleNew={handleAddMarker} />
          <MapMarkerList
            markers={mapData.markers}
            removeMarker={handleRemoveMarker}
          />
        </div>
        <div>
          <MapPerimeterInput handleNew={handleAddPerimeter} />
          <MapPerimeterList
            perimeter={mapData.perimeter}
            removePerimeter={handleRemovePerimeter}
          />
        </div>
      </div>
    </div>
  );
}
