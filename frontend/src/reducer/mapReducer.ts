import { MapData } from '../types';

export type MapReducerState = {
  marker: Array<MapData>;
  waypoint: Array<MapData>;
  perimeter: Array<MapData>;
  explored: Array<MapData>;
};

type MapReducerAction = {
  type: string;
  payload: MapData;
};

export default function mapReducer(state: MapReducerState, action: any) {
  switch (action.type) {
    case 'marker_added': {
      return {
        ...state,
        marker: [...state.marker, action.payload],
      };
    }
    case 'marker_removed': {
      return {
        ...state,
        marker: state.marker.filter(
          (marker) => marker.id !== action.payload.id,
        ),
      };
    }
    case 'waypoint_added': {
      return {
        ...state,
        waypoint: [...state.waypoint, action.payload],
      };
    }
    case 'waypoint_removed': {
      return {
        ...state,
        waypoint: state.waypoint.filter(
          (waypoint) => waypoint.id !== action.payload.id,
        ),
      };
    }
    case 'perimeter_added': {
      return {
        ...state,
        perimeter: [...state.perimeter, action.payload],
      };
    }
    case 'perimeter_removed': {
      return {
        ...state,
        perimeter: state.perimeter.filter(
          (perimeter) => perimeter.id !== action.payload.id,
        ),
      };
    }
    case 'explored_added': {
      return {
        ...state,
        explored: [...state.explored, action.payload],
      };
    }
    case 'explored_removed': {
      return {
        ...state,
        explored: state.explored.filter(
          (explored) => explored.id !== action.payload.id,
        ),
      };
    }
    case 'set_all': {
      return {
        explored: action.payload.explored,
        perimeter: action.payload.perimeter,
        waypoint: action.payload.waypoint,
        marker: action.payload.marker,
      };
    }
  }
  throw new Error('Invalid action type');
}
