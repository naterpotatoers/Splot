import axios from 'axios';
import { MapData } from '../types';
import { BASE_URL } from '../utils/constants';

export async function getMarkers(): Promise<MapData[]> {
  const url = `${BASE_URL}/markers`;
  const response = await axios.get(url);
  return response.data;
}

export async function addMarker(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/markers`;
  const response = await axios.post(url, mapData);
  return response.data;
}

export async function removeMarker(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/markers/${mapData.id}`;
  const response = await axios.delete(url, { data: mapData });
  return response.data;
}

export async function removeAllMarkers(): Promise<Array<MapData>> {
  const url = `${BASE_URL}/markers/`;
  const response = await axios.delete(url);
  return response.data;
}
