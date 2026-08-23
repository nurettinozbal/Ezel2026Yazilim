import { describe, it, expect } from 'vitest';
import { isInsidePolygon, polygonCentroid, toLatLngPairs } from '../geofence';

// Basit kare bölge: lat 40..41, lon 28..29
const SQUARE = [
  { lat: 40, lon: 28 },
  { lat: 41, lon: 28 },
  { lat: 41, lon: 29 },
  { lat: 40, lon: 29 },
];

describe('isInsidePolygon', () => {
  it('bölge içindeki noktayı true döner', () => {
    expect(isInsidePolygon(40.5, 28.5, SQUARE)).toBe(true);
  });

  it('bölge dışındaki noktayı false döner', () => {
    expect(isInsidePolygon(42, 28.5, SQUARE)).toBe(false);
    expect(isInsidePolygon(40.5, 30, SQUARE)).toBe(false);
  });

  it('3ten az nokta varsa null döner (sınır tanımsız)', () => {
    expect(isInsidePolygon(40.5, 28.5, [])).toBeNull();
    expect(isInsidePolygon(40.5, 28.5, [{ lat: 40, lon: 28 }, { lat: 41, lon: 28 }])).toBeNull();
  });

  it('geçersiz/eksik konum için null döner', () => {
    expect(isInsidePolygon(NaN, 28.5, SQUARE)).toBeNull();
    expect(isInsidePolygon(undefined, undefined, SQUARE)).toBeNull();
  });

  it('GPS fix yokken gelen (0,0) konumunu ihlal saymaz', () => {
    expect(isInsidePolygon(0, 0, SQUARE)).toBeNull();
  });

  it('bozuk poligon noktası varsa null döner', () => {
    const broken = [...SQUARE.slice(0, 3), { lat: null, lon: 29 }];
    expect(isInsidePolygon(40.5, 28.5, broken)).toBeNull();
  });
});

describe('polygonCentroid', () => {
  it('kare bölgenin merkezini bulur', () => {
    expect(polygonCentroid(SQUARE)).toEqual({ lat: 40.5, lon: 28.5 });
  });

  it('boş liste için null döner', () => {
    expect(polygonCentroid([])).toBeNull();
    expect(polygonCentroid(null)).toBeNull();
  });
});

describe('toLatLngPairs', () => {
  it('Leaflet biçimine çevirir ve geçersizleri atar', () => {
    expect(toLatLngPairs([{ lat: 1, lon: 2 }, { lat: null, lon: 3 }])).toEqual([[1, 2]]);
  });

  it('tanımsız girdi için boş dizi döner', () => {
    expect(toLatLngPairs(undefined)).toEqual([]);
  });
});
