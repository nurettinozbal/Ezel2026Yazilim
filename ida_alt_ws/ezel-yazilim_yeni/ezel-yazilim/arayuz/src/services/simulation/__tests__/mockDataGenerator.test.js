import { describe, it, expect } from 'vitest';
import { generateMockData } from '@/services/simulation/mockDataGenerator';

describe('mockDataGenerator', () => {
  it('generates valid telemetry structure', () => {
    const data = generateMockData();
    
    // Üst seviye anahtarlar kontrol
    expect(data).toHaveProperty('ida');
    expect(data).toHaveProperty('iha');
    expect(data).toHaveProperty('system');
  });

  it('ida telemetry returns correct types', () => {
    const { ida } = generateMockData();

    expect(typeof ida.sys_id).toBe('number');
    expect(typeof ida.speed).toBe('number');
    expect(typeof ida.voltage).toBe('number');
    expect(typeof ida.current).toBe('number');
    expect(typeof ida.lat).toBe('number');
    expect(typeof ida.lon).toBe('number');
    expect(typeof ida.heading).toBe('number');
    expect(typeof ida.battery_percent).toBe('number');
    expect(typeof ida.armed).toBe('boolean');
    expect(typeof ida.connected).toBe('boolean');
  });

  it('iha telemetry returns correct types', () => {
    const { iha } = generateMockData();

    expect(typeof iha.battery).toBe('number');
    expect(typeof iha.alt).toBe('number');
    expect(typeof iha.lat).toBe('number');
    expect(typeof iha.lon).toBe('number');
    expect(typeof iha.armed).toBe('boolean');
    expect(typeof iha.connected).toBe('boolean');
    expect(typeof iha.detected_color).toBe('string');
  });

  it('system telemetry returns correct types', () => {
    const { system } = generateMockData();

    expect(typeof system.rssi).toBe('number');
    expect(typeof system.gps_sats).toBe('number');
    expect(typeof system.hdop).toBe('number');
    expect(typeof system.failsafe_status).toBe('string');
    expect(typeof system.ida_link).toBe('string');
    expect(typeof system.iha_link).toBe('string');
  });

  it('voltage values are within realistic range', () => {
    const { ida } = generateMockData();
    expect(ida.voltage).toBeGreaterThanOrEqual(16.0);
    expect(ida.voltage).toBeLessThanOrEqual(17.1);
  });

  it('coordinates are near expected base location', () => {
    const { ida, iha } = generateMockData();
    // İstanbul Boğazı civarı
    expect(ida.lat).toBeCloseTo(41.025, 2);
    expect(ida.lon).toBeCloseTo(28.974, 2);
    expect(iha.lat).toBeCloseTo(41.026, 2);
    expect(iha.lon).toBeCloseTo(28.975, 2);
  });

  it('detected_color is one of valid values', () => {
    const { iha } = generateMockData();
    expect(['KIRMIZI', 'YEŞİL', 'SİYAH', 'YOK']).toContain(iha.detected_color);
  });

  it('heading is within 0-360 range', () => {
    const { ida } = generateMockData();
    expect(ida.heading).toBeGreaterThanOrEqual(0);
    expect(ida.heading).toBeLessThan(360);
  });
});
