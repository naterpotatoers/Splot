import { addExplored, getExplored } from '../api/explored';
import { addMarker, getMarkers } from '../api/marker';
import { addPerimeter, getPerimeters } from '../api/perimeter';
import { addSplotWaypoint, getScoutWaypoints } from '../api/waypoint';
import { MapReducerState } from '../reducer/mapReducer';
import { ClickStatus, MapData } from '../types';

export async function addMapData(
  clickStatus: ClickStatus,
  mapData: MapData,
) {
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
