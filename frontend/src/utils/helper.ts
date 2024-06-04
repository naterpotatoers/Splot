import { addExplored, getExplored, removeAllExplored } from '../api/explored';
import { addMarker, getMarkers, removeAllMarkers } from '../api/marker';
import {
  addPerimeter,
  getPerimeters,
  removeAllPerimeter,
} from '../api/perimeter';
import {
  addSplotWaypoint,
  getScoutWaypoints,
  removeAllScoutWaypoints,
} from '../api/waypoint';
import { MapReducerState } from '../reducer/mapReducer';
import { ClickStatus, MapData } from '../types';

export async function addMapData(clickStatus: ClickStatus, mapData: MapData) {
  switch (clickStatus) {
    case 'perimeter':
      await addPerimeter(mapData);
      break;
    case 'marker':
      await addMarker(mapData);
      break;
    case 'explored':
      await addExplored(mapData);
      break;
    case 'waypoint':
      await addSplotWaypoint(mapData);
      break;
    default:
      break;
  }
}

export async function getAllMapPositions(): Promise<MapReducerState> {
  const explored = await getExplored();
  const perimeter = await getPerimeters();
  const waypoint = await getScoutWaypoints();
  const marker = await getMarkers();
  return {
    explored,
    perimeter,
    waypoint,
    marker,
  };
}

export async function removeAllMapPositions(): Promise<MapReducerState> {
  const explored = await removeAllExplored();
  const perimeter = await removeAllPerimeter();
  const waypoint = await removeAllScoutWaypoints();
  const marker = await removeAllMarkers();

  return {
    explored,
    perimeter,
    waypoint,
    marker,
  };
}
