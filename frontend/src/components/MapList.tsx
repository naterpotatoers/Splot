import { Trash2 } from 'lucide-react';
import { MapData } from '../types';

type MapListProps = {
  label: string;
  list: Array<MapData>;
  remove: (list: MapData) => void;
};

export default function MapList({ label, list, remove }: MapListProps) {
  return (
    <>
      <h2>{label} List</h2>
      {list.map((data: MapData) => (
        <div key={data.id} className="grid-col-4 gap">
          <p>{data.id}</p>
          <p>
            {data.coords.lat},{data.coords.lng}
          </p>
          <p>{data.desc}</p>
          <button
            data-testid={`${label.toLowerCase()}-delete-button`}
            onClick={() => remove(data)}
          >
            <Trash2 />
          </button>
        </div>
      ))}
    </>
  );
}
