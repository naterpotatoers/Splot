import { expect, it, describe } from 'vitest';

import { MOCK_MAP_EXPLORED, MOCK_MAP_PERIMETER } from '../utils/constants';
import {
  getMapCenter,
  getPerimeterCoordinates,
  getUniqueDeviceId,
  groupCoordinatesById,
} from '../utils';

describe('Utils', () => {
  it('should return the center of the coordinates', () => {
    const result = getMapCenter(MOCK_MAP_PERIMETER);
    const expected = { lat: 36.74, lng: -119.72 };
    expect(result).toEqual(expected);
  });

  it('should return the perimeter coordinates', () => {
    const result = getPerimeterCoordinates(MOCK_MAP_PERIMETER);
    const expected = [
      { lat: 36.77, lng: -119.74 },
      { lat: 36.71, lng: -119.71 },
    ];
    expect(result).toEqual(expected);

    const result2 = getPerimeterCoordinates([]);
    const expected2 = [{ lat: 0, lng: 0 }];
    expect(result2).toEqual(expected2);
  });

  it('should return the unique ids', () => {
    const result = getUniqueDeviceId(MOCK_MAP_EXPLORED);
    const expected = ['drone-x', 'drone-y', 'drone-z'];
    expect(result).toEqual(expected);
  });

  it('should return the coordinates grouped by id', () => {
    const result = groupCoordinatesById(MOCK_MAP_EXPLORED);
    const expected = [
      [
        MOCK_MAP_EXPLORED[0].coords,
        MOCK_MAP_EXPLORED[1].coords,
        MOCK_MAP_EXPLORED[2].coords,
      ],
      [
        MOCK_MAP_EXPLORED[3].coords,
        MOCK_MAP_EXPLORED[4].coords,
        MOCK_MAP_EXPLORED[5].coords,
      ],
      [
        MOCK_MAP_EXPLORED[6].coords,
        MOCK_MAP_EXPLORED[7].coords,
        MOCK_MAP_EXPLORED[8].coords,
      ],
    ];
    expect(result).toEqual(expected);
  });
});
