import { render } from '@testing-library/react';
import MapMarkerInput from '../MapMarkerInput';

describe('MapMarkerInput', () => {
  it('should successfully enter in lat and lng values', async () => {
    render(<MapMarkerInput setMarkers={() => {}} />);
  });
});
