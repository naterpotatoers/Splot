import axios from 'axios';
import { MapData } from '../types';
import { BASE_URL } from '../utils/constants';

export async function getPerimeters(): Promise<MapData[]> {
  const url = `${BASE_URL}/perimeter`;
  const response = await axios.get(url);
  return response.data;
}

export async function addPerimeter(mapData: MapData): Promise<MapData[]> {
  const url = `${BASE_URL}/perimeter`;
  const response = await axios.post(url, mapData);
  return response.data;
}

export async function removePerimeter(mapData: MapData): Promise<MapData[]> {
  const url = `${BASE_URL}/perimeter/${mapData.id}`;
  const response = await axios.delete(url, { data: mapData });
  return response.data;
}

export async function removeAllPerimeter(): Promise<MapData[]> {
  const url = `${BASE_URL}/perimeter`;
  const response = await axios.delete(url);
  return response.data;
}
