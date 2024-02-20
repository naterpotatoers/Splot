import { Trash2 } from 'lucide-react';
import React from 'react';

type MapPerimeterListProps = {
  perimeter: Array<any>;
  removePerimeter: (perimeter: any) => void;
};

export default function MapPerimeterList({
  perimeter,
  removePerimeter,
}: MapPerimeterListProps) {
  return (
    <>
      <h2>Map Perimeter List</h2>
      {perimeter.map((perimeter: any) => (
        <div key={perimeter.id} className="flex-row">
          <p>{perimeter.id}</p>
          <p>
            {perimeter.coords.lat},{perimeter.coords.lng}
          </p>
          <p>{perimeter.desc}</p>
          <button
            data-testid="perimeter-delete-button"
            onClick={() => removePerimeter(perimeter)}
          >
            <Trash2 />
          </button>
        </div>
      ))}
    </>
  );
}
