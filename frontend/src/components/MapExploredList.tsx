import { Trash2 } from 'lucide-react';
import React from 'react';

type MapExploredListProps = {
  explored: Array<any>;
  removeExplored: (explored: any) => void;
};

export default function MapExploredList({
  explored,
  removeExplored,
}: MapExploredListProps) {
  return (
    <>
      <h2>Map Explored List</h2>
      {explored.map((explored: any) => (
        <div key={explored.id} className="flex-row">
          <p>{explored.id}</p>
          <p>
            {explored.coords.lat},{explored.coords.lng}
          </p>
          <p>{explored.desc}</p>
          <button
            data-testid="explored-delete-button"
            onClick={() => removeExplored(explored)}
          >
            <Trash2 />
          </button>
        </div>
      ))}
    </>
  );
}
