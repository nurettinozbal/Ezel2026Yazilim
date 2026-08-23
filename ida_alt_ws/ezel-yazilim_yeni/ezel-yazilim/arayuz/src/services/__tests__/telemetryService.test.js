import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import vehicleTestContract from '../../../contracts/ida_vehicle_test.v1.json';

/**
 * TelemetryService unit tests.
 * Tests transport logic, mock mode, and WebSocket reconnection behavior.
 * Does NOT require a real WebSocket server — mocks are used.
 */

// Mock the import so we can control behavior
vi.mock('@/services/simulation/mockDataGenerator', () => ({
  generateMockData: vi.fn(() => ({
    ida: { sys_id: 1, speed: 2.0, voltage: 16.5, lat: 41.025, lon: 28.974 },
    iha: { battery: 85, alt: 12.5, lat: 41.026, lon: 28.975 },
    system: { rssi: -45, gps_sats: 14, hdop: 0.8 },
  })),
}));

describe('TelemetryService', () => {
  let telemetryService;

  beforeEach(async () => {
    vi.stubEnv('VITE_USE_MOCK', 'true');
    vi.useFakeTimers();
    // Re-import to get fresh module state
    const module = await import('@/services/telemetryService');
    telemetryService = module.telemetryService;
  });

  afterEach(() => {
    telemetryService.disconnect();
    vi.useRealTimers();
    vi.unstubAllEnvs();
    vi.unstubAllGlobals();
  });

  it('starts in mock mode when VITE_USE_MOCK is true', () => {
    const onTelemetry = vi.fn();
    const onStatus = vi.fn();
    const onLog = vi.fn();

    telemetryService.connect({
      onTelemetry,
      onLog,
      onStatus,
      onCommandResult: vi.fn(),
    });

    // İlk telemetry hemen gelmeli
    expect(onTelemetry).toHaveBeenCalledTimes(1);
    expect(onStatus).toHaveBeenCalledWith('MOCK');
  });

  it('sends periodic telemetry in mock mode', () => {
    const onTelemetry = vi.fn();

    telemetryService.connect({
      onTelemetry,
      onLog: vi.fn(),
      onStatus: vi.fn(),
      onCommandResult: vi.fn(),
    });

    // İlk çağrı + 500ms sonra bir tane daha
    expect(onTelemetry).toHaveBeenCalledTimes(1);
    vi.advanceTimersByTime(500);
    expect(onTelemetry).toHaveBeenCalledTimes(2);
    vi.advanceTimersByTime(500);
    expect(onTelemetry).toHaveBeenCalledTimes(3);
  });

  it('sendCommand returns true and simulates result in mock mode', () => {
    const onCommandResult = vi.fn();

    telemetryService.connect({
      onTelemetry: vi.fn(),
      onLog: vi.fn(),
      onStatus: vi.fn(),
      onCommandResult,
    });

    const result = telemetryService.sendCommand('ARM_IDA');
    expect(result).toBe(true);
    expect(onCommandResult).toHaveBeenCalledWith(
      expect.objectContaining({
        ok: true,
        command: 'ARM_IDA',
      })
    );
  });

  it('stops sending telemetry after disconnect', () => {
    const onTelemetry = vi.fn();

    telemetryService.connect({
      onTelemetry,
      onLog: vi.fn(),
      onStatus: vi.fn(),
      onCommandResult: vi.fn(),
    });

    telemetryService.disconnect();
    const countAfterDisconnect = onTelemetry.mock.calls.length;

    vi.advanceTimersByTime(2000);
    expect(onTelemetry).toHaveBeenCalledTimes(countAfterDisconnect);
  });

  it('routes versioned debug snapshots and passive test events', () => {
    const onDebugSnapshot = vi.fn();
    const onVehicleTest = vi.fn();
    telemetryService.connect({
      onTelemetry: vi.fn(), onLog: vi.fn(), onStatus: vi.fn(), onCommandResult: vi.fn(),
      onDebugSnapshot, onVehicleTest,
    });
    telemetryService.handleMessage(JSON.stringify({ type: 'debug_snapshot', data: { schema_version: 1, lidar: { status: 'unknown' } } }));
    telemetryService.handleMessage(JSON.stringify({ type: 'vehicle_test', data: { schema_version: 1, event: 'manifest', tests: [] } }));
    telemetryService.handleMessage(JSON.stringify({ type: 'debug_snapshot', data: { schema_version: 99 } }));
    expect(onDebugSnapshot).toHaveBeenCalledTimes(1);
    expect(onVehicleTest).toHaveBeenCalledTimes(1);
  });

  it('uses canonical ida_vehicle_test manifest ids', () => {
    expect(vehicleTestContract.tests.map((test) => test.id)).toEqual([
      'comms', 'telemetry', 'camera_p1p2', 'camera_p3', 'lidar',
      'fusion_shadow', 'autonomy_shadow', 'logging',
    ]);
    expect(vehicleTestContract.events).toContain('result');
    expect(vehicleTestContract.actions).toContain('cancel');
    expect(vehicleTestContract.tests.find((test) => test.id === 'lidar')).toMatchObject({
      requires_expectations: true, default_profile: 'lidar_bottle', timeout_s: 20,
    });
  });

  it('sends auth token when backend mode opens a WebSocket', () => {
    vi.stubEnv('VITE_USE_MOCK', 'false');
    vi.stubEnv('VITE_WS_TOKEN', 'field-token');
    const sentMessages = [];
    const openedUrls = [];

    class FakeWebSocket {
      static CONNECTING = 0;
      static OPEN = 1;

      constructor(url) {
        openedUrls.push(url);
        this.readyState = FakeWebSocket.CONNECTING;
        setTimeout(() => {
          this.readyState = FakeWebSocket.OPEN;
          this.onopen?.();
        }, 0);
      }

      send(message) {
        sentMessages.push(JSON.parse(message));
      }

      close() {
        this.readyState = 3;
      }
    }

    vi.stubGlobal('WebSocket', FakeWebSocket);

    telemetryService.connect({
      onTelemetry: vi.fn(),
      onLog: vi.fn(),
      onStatus: vi.fn(),
      onCommandResult: vi.fn(),
    });
    vi.advanceTimersByTime(0);

    expect(openedUrls[0]).toBe('ws://localhost:5000/ws');
    expect(sentMessages).toContainEqual({
      type: 'auth',
      token: 'field-token',
    });
    const expectations = {
      profile: 'lidar_bottle', case: 'positive', expected_color: null,
      expected_range_m: 2, expected_bearing_deg: 5,
      range_tolerance_m: 0.25, bearing_tolerance_deg: 3, expected_id: 'bottle-1',
    };
    expect(telemetryService.sendVehicleTest('start', { test_id: 'lidar', timeout_s: 20, expectations })).toBe(true);
    expect(sentMessages).toContainEqual({
      type: 'vehicle_test', data: { action: 'start', test_id: 'lidar', timeout_s: 20, expectations },
    });
  });
});
