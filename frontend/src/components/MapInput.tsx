import { Plus } from 'lucide-react';
import React from 'react';
import { MapData } from '../types';
import { handleMapInputSubmit } from '../utils';

type MapInputProps = {
  label: string;
  create: (data: MapData) => void;
};

export default function MapInput({ label, create }: MapInputProps) {
  const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
    const newData = handleMapInputSubmit(e, label);
    create(newData);
  };

  return (
    <>
      <h3>Add {label}</h3>
      <form className="grid-col-4" onSubmit={handleSubmit}>
        <div className="form-field">
          <label>Latitude</label>
          <input
            type="number"
            name="lat"
            data-testid={`${label.toLowerCase()}-lat-input`}
            step={0.01}
            max={90}
            min={-90}
            required
          />
        </div>
        <div className="form-field">
          <label>Longitude</label>
          <input
            type="number"
            name="lng"
            data-testid={`${label.toLowerCase()}-lng-input`}
            step={0.01}
            max={180}
            min={-180}
            required
          />
        </div>
        <div className="form-field">
          <label>Description</label>
          <input
            type="text"
            name="desc"
            data-testid={`${label.toLowerCase()}-desc-input`}
            required
          />
        </div>
        <button
          type="submit"
          data-testid={`${label.toLowerCase()}-submit-button`}
        >
          <Plus />
        </button>
      </form>
    </>
  );
}
