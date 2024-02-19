import { Plus } from 'lucide-react';
import React from 'react';
import { MapMarker } from '../types';

type MapMarkerInputProps = {
  handleNew: (newMarker: MapMarker) => void;
};

export default function MapMarkerInput({ handleNew }: MapMarkerInputProps) {
  const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const form = e.currentTarget;
    const lat = parseFloat(form.lat.value);
    const lng = parseFloat(form.lng.value);
    const desc = form.desc.value;
    const newMarker: MapMarker = {
      id: Date.now().toString(),
      coords: { lat, lng },
      desc,
    };
    handleNew(newMarker);
    form.reset();
  };

  return (
    <form className="flex-row padding-10" onSubmit={handleSubmit}>
      <label>Latitude</label>
      <input
        type="number"
        name="lat"
        testdata-id="lat-input"
        step={0.01}
        max={90}
        min={-90}
        required
      />
      <label>Longitude</label>
      <input
        type="number"
        name="lng"
        testdata-id="lng-input"
        step={0.01}
        max={180}
        min={-180}
        required
      />
      <label>Description</label>
      <input type="text" name="desc" testdata-id="desc-input" required />
      <button type="submit" testdata-id="submit-marker-input">
        <Plus />
      </button>
    </form>
  );
}
