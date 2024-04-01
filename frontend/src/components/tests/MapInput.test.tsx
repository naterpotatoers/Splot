import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event'
import '@testing-library/jest-dom'

import { MOCK_MAP_MARKERS } from '../../utils/constants';

import MapInput from '../MapInput';

describe('Map Input', () => {
  it('should render input and create new entry', async () => {
    const label = 'Test';
    const INPUT = MOCK_MAP_MARKERS[0];
    render(<MapInput label={label} create={() => {}} />);
    await screen.findByRole('heading', { name: `Add ${label}` });
    const latInput = await screen.findByTestId(`${label.toLowerCase()}-lat-input`);
    const lngInput = await screen.findByTestId(`${label.toLowerCase()}-lng-input`);
    const descInput = await screen.findByTestId(`${label.toLowerCase()}-desc-input`);
    const submitButton = await screen.findByTestId(`${label.toLowerCase()}-submit-button`);

    userEvent.type(latInput, INPUT.coords.lat.toString());
    userEvent.type(lngInput, INPUT.coords.lng.toString());
    userEvent.type(descInput, INPUT.desc);
    userEvent.click(submitButton);
  });
});
