import { MapData } from '../types';
import MapList from './MapList';
import MapInput from './MapInput';
import { addPerimeter, removePerimeter } from '../api/perimeter';
import { addExplored, removeExplored } from '../api/explored';
import { addScoutWaypoint, removeScoutWaypoint } from '../api/waypoint';
import { addMarker, removeMarker } from '../api/marker';
import { MapReducerState } from '../reducer/mapReducer';

type MapFormsProps = {
  mapData: MapReducerState;
  dispatch: any;
};

export default function MapForm({ mapData, dispatch }: MapFormsProps) {
  const handleAddPerimeter = (mapData: MapData) => {
    dispatch({ type: 'perimeter_added', payload: mapData });
    addPerimeter(mapData);
  };

  const handleAddMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_added', payload: mapData });
    addMarker(mapData);
  };

  const handleRemoveMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_removed', payload: mapData });
    removeMarker(mapData);
  };

  const handleRemovePerimeter = (mapData: MapData) => {
    dispatch({ type: 'perimeter_removed', payload: mapData });
    removePerimeter(mapData);
  };

  const handleAddExplored = (mapData: MapData) => {
    dispatch({ type: 'explored_added', payload: mapData });
    addExplored(mapData);
  };

  const handleRemoveExplored = (mapData: MapData) => {
    dispatch({ type: 'explored_removed', payload: mapData });
    removeExplored(mapData);
  };

  const handleAddWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_added', payload: mapData });
    addScoutWaypoint(mapData);
  };

  const handleRemoveWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_removed', payload: mapData });
    removeScoutWaypoint(mapData);
  };

  return (
    <div className="grid-col-3 gap">
      <div id="map-marker-data">
        <MapInput label="Interest Marker" create={handleAddMarker} />
        <MapList
          label="Interest Marker"
          list={mapData.marker}
          remove={handleRemoveMarker}
        />
      </div>
      <div id="map-waypoint-data">
        <MapInput label="Waypoint" create={handleAddWaypoint} />
        <MapList
          label="Waypoint"
          list={mapData.waypoint}
          remove={handleRemoveWaypoint}
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
  );
}
