import axios from 'axios';
import { MapData } from '../types';
import { BASE_URL } from '../utils/constants';

export async function getExplored(): Promise<MapData[]> {
  const url = `${BASE_URL}/explored`;
  const response = await axios.get(url);
  return response.data;
}

export async function addExplored(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/explored`;
  const response = await axios.post(url, mapData);
  return response.data;
}

export async function removeExplored(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/explored/${mapData.id}`;
  const response = await axios.delete(url, { data: mapData });
  return response.data;
}
