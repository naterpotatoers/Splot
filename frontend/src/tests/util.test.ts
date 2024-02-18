import { expect, it, describe } from "vitest";

import { MAP_MARKERS } from "../constants";
import { getMapCenter } from "../utils";

describe("Utils", () => {
  it("should return the center of the coordinates", () => {
    const result = getMapCenter(MAP_MARKERS);
    const expected = MAP_MARKERS[1].coords;
    expect(result).toEqual(expected);
  });
});
