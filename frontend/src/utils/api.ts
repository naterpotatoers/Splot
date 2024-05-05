import axios from 'axios';
import { MapReducerState } from '../reducer/mapReducer';
import { MapData } from '../types';

export async function getExploredPositions(): Promise<MapData[]> {
  const response = await axios.get('http://localhost:5000/splot/explored');
  return response.data;
}

export async function getPerimeterPositions(): Promise<MapData[]> {
  const response = await axios.get('http://localhost:5000/splot/perimeter');
  return response.data;
}

export async function getWaypointPositions(): Promise<MapData[]> {
  const response = await axios.get('http://localhost:5000/splot/waypoint');
  return response.data;
}

export async function getMarkerPositions(): Promise<MapData[]> {
  const response = await axios.get('http://localhost:5000/splot/marker');
  return response.data;
}

export async function getAllMapPositions(): Promise<MapReducerState> {
  const explored = await getExploredPositions();
  const perimeter = await getPerimeterPositions();
  const waypoint = await getWaypointPositions();
  const marker = await getMarkerPositions();
  return {
    explored,
    perimeter,
    waypoint,
    marker,
  };
}
