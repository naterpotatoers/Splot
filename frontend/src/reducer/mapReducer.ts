import { MapData } from '../types';

type MapReducerState = {
  markers: Array<MapData>;
  perimeter: Array<MapData>;
  explored: Array<MapData>;
};

type MapReducerAction = {
  type: string;
  payload: MapData;
};

export default function mapReducer(
  state: MapReducerState,
  action: MapReducerAction,
) {
  switch (action.type) {
    case 'marker_added': {
      return {
        ...state,
        markers: [...state.markers, action.payload],
      };
    }
    case 'marker_removed': {
      return {
        ...state,
        markers: state.markers.filter(
          (marker) => marker.id !== action.payload.id,
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
  }
  throw Error('Unknown action: ' + action.type);
}
