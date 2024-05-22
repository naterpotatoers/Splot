import axios from 'axios';
import { MapData } from '../types';
import { BASE_URL } from '../utils/constants';

export async function getSplotWaypoints(): Promise<MapData[]> {
  const url = `${BASE_URL}/splot/waypoints`;
  const response = await axios.get(url);
  return response.data;
}

export async function addSplotWaypoint(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/splot/waypoints`;
  const response = await axios.post(url, mapData);
  return response.data;
}

export async function removeSplotWaypoint(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/splot/waypoints/${mapData.id}`;
  const response = await axios.delete(url, { data: mapData });
  return response.data;
}

export async function removeAllSplotWaypoints(): Promise<MapData> {
  const url = `${BASE_URL}/splot/waypoints`;
  const response = await axios.delete(url);
  return response.data;
}

export async function getScoutWaypoints(): Promise<MapData[]> {
  const url = `${BASE_URL}/scout/waypoints`;
  const response = await axios.get(url);
  return response.data;
}

export async function addScoutWaypoint(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/scout/waypoints`;
  const response = await axios.post(url, mapData);
  return response.data;
}

export async function removeScoutWaypoint(mapData: MapData): Promise<MapData> {
  const url = `${BASE_URL}/scout/waypoints/${mapData.id}`;
  const response = await axios.delete(url, { data: mapData });
  return response.data;
}

export async function removeAllScoutWaypoints(): Promise<MapData> {
  const url = `${BASE_URL}/scout/waypoints`;
  const response = await axios.delete(url);
  return response.data;
}
