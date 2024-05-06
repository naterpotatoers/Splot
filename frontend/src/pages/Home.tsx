import Map from '../components/Map';
import { ClickStatus } from '../types';
import Header from '../components/Header';
import MapForm from '../components/MapForm';
import mapReducer from '../reducer/mapReducer';
import { useQuery } from '@tanstack/react-query';
import { getAllMapPositions } from '../utils/helper';
import { DEFAULT_MAP_DATA } from '../utils/constants';
import { useEffect, useReducer, useState } from 'react';

export default function Home() {
  const [clickStatus, setClickStatus] = useState<ClickStatus>('perimeter');
  const [mapData, dispatch] = useReducer(mapReducer, DEFAULT_MAP_DATA);

  const { data } = useQuery({
    queryKey: ['mapPositions'],
    queryFn: getAllMapPositions,
    refetchInterval: 1500,
  });

  useEffect(() => {
    if (data) {
      dispatch({ type: 'set_all', payload: data });
    }
  }, [data]);

  return (
    <div>
      <Header clickStatus={clickStatus} setClickStatus={setClickStatus} />
      <Map clickStatus={clickStatus} mapData={mapData} dispatch={dispatch} />
      <MapForm mapData={mapData} dispatch={dispatch} />
    </div>
  );
}
