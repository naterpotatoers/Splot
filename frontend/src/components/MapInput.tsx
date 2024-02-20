import { Plus } from 'lucide-react';
import React from 'react';
import { MapData } from '../types';

type MapInputProps = {
  label: string;
  create: (data: MapData) => void;
};

export default function MapInput({ label, create }: MapInputProps) {
  const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const form = e.currentTarget;
    const lat = parseFloat(form.lat.value);
    const lng = parseFloat(form.lng.value);
    const desc = form.desc.value;
    const newMarker: MapData = {
      id: `manually-${label.toLowerCase()}-` + Date.now().toString(),
      coords: { lat, lng },
      desc,
    };
    create(newMarker);
    form.reset();
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
