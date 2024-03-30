import {
  EXPLORED,
  HEADER,
  MARKER,
  PERIMETER,
  WAYPOINT,
} from '../support/elements';

const GENERIC_INPUT = {
  desc: 'test',
  coords: { lat: 36.68, lng: -119.8 },
};

describe('template spec', () => {
  beforeEach(() => {
    cy.visit('/');
    cy.contains('Splot');
  });

  it('should change the click status', () => {
    cy.get(HEADER.BUTTON.MARKER).click();
    cy.get(HEADER.STATUS).contains('marker');
    cy.get(HEADER.BUTTON.WAYPOINT).click();
    cy.get(HEADER.STATUS).contains('waypoint');
    cy.get(HEADER.BUTTON.PERIMETER).click();
    cy.get(HEADER.STATUS).contains('perimeter');
  });

  it('should add new marker to map', () => {
    cy.get(MARKER.INPUT.LAT).type(GENERIC_INPUT.coords.lat.toString());
    cy.get(MARKER.INPUT.LNG).type(GENERIC_INPUT.coords.lng.toString());
    cy.get(MARKER.INPUT.DESC).type(GENERIC_INPUT.desc);
    cy.get(MARKER.BUTTON.SUBMIT).click();

    cy.contains(GENERIC_INPUT.desc);
    cy.get(MARKER.BUTTON.DELETE).last().click();
    cy.contains(GENERIC_INPUT.desc).should('not.exist');
  });

  it('should add a new waypoint to map', () => {
    cy.get(WAYPOINT.INPUT.LAT).type(GENERIC_INPUT.coords.lat.toString());
    cy.get(WAYPOINT.INPUT.LNG).type(GENERIC_INPUT.coords.lng.toString());
    cy.get(WAYPOINT.INPUT.DESC).type(GENERIC_INPUT.desc);
    cy.get(WAYPOINT.BUTTON.SUBMIT).click();

    cy.contains(GENERIC_INPUT.desc);
    cy.get(WAYPOINT.BUTTON.DELETE).last().click();
    cy.contains(GENERIC_INPUT.desc).should('not.exist');
  });

  it('should add new perimeter to map', () => {
    cy.get(PERIMETER.INPUT.LAT).type(GENERIC_INPUT.coords.lat.toString());
    cy.get(PERIMETER.INPUT.LNG).type(GENERIC_INPUT.coords.lng.toString());
    cy.get(PERIMETER.INPUT.DESC).type(GENERIC_INPUT.desc);
    cy.get(PERIMETER.BUTTON.SUBMIT).click();

    cy.contains(GENERIC_INPUT.desc);
    cy.get(PERIMETER.BUTTON.DELETE).last().click();
    cy.contains(GENERIC_INPUT.desc).should('not.exist');
  });

  it('should add new explored to map', () => {
    const EXPLORED_INPUT_1 = {
      desc: 'manually set explored 1',
      coords: { lat: 36.71, lng: -119.71 },
    };

    const EXPLORED_INPUT_2 = {
      desc: 'manually set explored 2',
      coords: { lat: 36.72, lng: -119.74 },
    };

    cy.get(EXPLORED.INPUT.LAT).type(EXPLORED_INPUT_1.coords.lat.toString());
    cy.get(EXPLORED.INPUT.LNG).type(EXPLORED_INPUT_1.coords.lng.toString());
    cy.get(EXPLORED.INPUT.DESC).type(EXPLORED_INPUT_1.desc);
    cy.get(EXPLORED.BUTTON.SUBMIT).click();
    cy.contains(EXPLORED_INPUT_1.desc);

    cy.get(EXPLORED.INPUT.LAT).type(EXPLORED_INPUT_2.coords.lat.toString());
    cy.get(EXPLORED.INPUT.LNG).type(EXPLORED_INPUT_2.coords.lng.toString());
    cy.get(EXPLORED.INPUT.DESC).type(EXPLORED_INPUT_2.desc);
    cy.get(EXPLORED.BUTTON.SUBMIT).click();
    cy.contains(EXPLORED_INPUT_2.desc);

    cy.get(EXPLORED.INPUT.LAT).type(GENERIC_INPUT.coords.lat.toString());
    cy.get(EXPLORED.INPUT.LNG).type(GENERIC_INPUT.coords.lng.toString());
    cy.get(EXPLORED.INPUT.DESC).type(GENERIC_INPUT.desc);
    cy.get(EXPLORED.BUTTON.SUBMIT).click();
    cy.contains(GENERIC_INPUT.desc);

    cy.get(EXPLORED.BUTTON.DELETE).last().click();
    cy.contains(GENERIC_INPUT.desc).should('not.exist');
  });
});
