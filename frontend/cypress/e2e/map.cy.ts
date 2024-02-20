import { EXPLORED, MARKER, PERIMETER } from '../support/elements';

const TEST_INPUT = {
  desc: 'test',
  coords: { lat: 36.68, lng: -119.8 },
};

describe('template spec', () => {
  beforeEach(() => {
    cy.visit('/');
    cy.contains('Splot');
  });

  it('should add new marker to map', () => {
    cy.get(MARKER.INPUT.LAT).type(TEST_INPUT.coords.lat.toString());
    cy.get(MARKER.INPUT.LNG).type(TEST_INPUT.coords.lng.toString());
    cy.get(MARKER.INPUT.DESC).type(TEST_INPUT.desc);
    cy.get(MARKER.BUTTON.SUBMIT).click();

    cy.contains(TEST_INPUT.desc);
    cy.get(MARKER.BUTTON.DELETE).last().click();
    cy.contains(TEST_INPUT.desc).should('not.exist');
  });

  it('should add new perimeter to map', () => {
    cy.get(PERIMETER.INPUT.LAT).type(TEST_INPUT.coords.lat.toString());
    cy.get(PERIMETER.INPUT.LNG).type(TEST_INPUT.coords.lng.toString());
    cy.get(PERIMETER.INPUT.DESC).type(TEST_INPUT.desc);
    cy.get(PERIMETER.BUTTON.SUBMIT).click();

    cy.contains(TEST_INPUT.desc);
    cy.get(PERIMETER.BUTTON.DELETE).last().click();
    cy.contains(TEST_INPUT.desc).should('not.exist');
  });

  it('should add new explored to map', () => {
    const INPUT_1 = {
      desc: 'manually set explored 1',
      coords: { lat: 36.71, lng: -119.71 },
    };

    const INPUT_2 = {
      desc: 'manually set explored 2',
      coords: { lat: 36.72, lng: -119.74 },
    };

    cy.get(EXPLORED.INPUT.LAT).type(INPUT_1.coords.lat.toString());
    cy.get(EXPLORED.INPUT.LNG).type(INPUT_1.coords.lng.toString());
    cy.get(EXPLORED.INPUT.DESC).type(INPUT_1.desc);
    cy.get(EXPLORED.BUTTON.SUBMIT).click();
    cy.contains(INPUT_1.desc);

    cy.get(EXPLORED.INPUT.LAT).type(INPUT_2.coords.lat.toString());
    cy.get(EXPLORED.INPUT.LNG).type(INPUT_2.coords.lng.toString());
    cy.get(EXPLORED.INPUT.DESC).type(INPUT_2.desc);
    cy.get(EXPLORED.BUTTON.SUBMIT).click();
    cy.contains(INPUT_2.desc);

    cy.get(EXPLORED.INPUT.LAT).type(TEST_INPUT.coords.lat.toString());
    cy.get(EXPLORED.INPUT.LNG).type(TEST_INPUT.coords.lng.toString());
    cy.get(EXPLORED.INPUT.DESC).type(TEST_INPUT.desc);
    cy.get(EXPLORED.BUTTON.SUBMIT).click();
    cy.contains(TEST_INPUT.desc);

    cy.get(EXPLORED.BUTTON.DELETE).last().click();
    cy.contains(TEST_INPUT.desc).should('not.exist');
  });
});
