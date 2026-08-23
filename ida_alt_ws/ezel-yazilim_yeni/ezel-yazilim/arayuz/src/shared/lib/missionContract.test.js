import { describe, expect, it } from 'vitest';
import { buildMissionWaypoints } from './missionContract';

describe('mission parkur contract', () => {
  it('preserves ordered P1/P2/P3 boundaries', () => {
    expect(buildMissionWaypoints([
      { lat: 41, lon: 29, parkur: 1 },
      { lat: 41.1, lon: 29.1, parkur: 1 },
      { lat: 41.2, lon: 29.2, parkur: 2 },
      { lat: 41.3, lon: 29.3, parkur: 3 },
    ])).toEqual([
      { lat: 41, lon: 29, alt: 0, parkur: 1 },
      { lat: 41.1, lon: 29.1, alt: 0, parkur: 1 },
      { lat: 41.2, lon: 29.2, alt: 0, parkur: 2 },
      { lat: 41.3, lon: 29.3, alt: 0, parkur: 3 },
    ]);
  });

  it('rejects missing and reversed parkur metadata', () => {
    expect(() => buildMissionWaypoints([{ lat: 41, lon: 29 }])).toThrow();
    expect(() => buildMissionWaypoints([
      { lat: 41, lon: 29, parkur: 2 },
      { lat: 41.1, lon: 29.1, parkur: 1 },
    ])).toThrow();
  });
});
