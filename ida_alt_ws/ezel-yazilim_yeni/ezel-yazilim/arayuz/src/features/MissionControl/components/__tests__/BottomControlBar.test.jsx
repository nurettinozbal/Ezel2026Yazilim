import { describe, expect, it, vi } from 'vitest';
import { fireEvent, render, screen } from '@testing-library/react';
import { VehicleContext } from '@/context/useVehicle';
import BottomControlBar from '@/features/MissionControl/components/BottomControlBar';

const renderBar = ({ missionStarted = false } = {}) => {
  const value = {
    telemetry: {
      ida: { armed: true, mode: 'GUIDED', voltage: 24.0, battery_percent: 80 },
      system: {
        mission_uploaded: true,
        mission_started: missionStarted,
        rssi: -55,
        hdop: 0.9,
        gps_sats: 15,
        failsafe_status: 'OK',
      },
    },
    isLogging: false,
    setIsLogging: vi.fn(),
    downloadLogs: vi.fn(),
    toggleArm: vi.fn(),
    startMission: vi.fn(),
    stopMission: vi.fn(),
  };
  render(
    <VehicleContext.Provider value={value}>
      <BottomControlBar />
    </VehicleContext.Provider>,
  );
  return value;
};

describe('BottomControlBar mission handshake controls', () => {
  it('starts only when an uploaded mission is ready', () => {
    const value = renderBar();

    fireEvent.click(screen.getByRole('button', { name: /GÖREV BAŞLAT/ }));

    expect(value.startMission).toHaveBeenCalledOnce();
    expect(value.stopMission).not.toHaveBeenCalled();
  });

  it('offers the verified stop path while mission is active', () => {
    const value = renderBar({ missionStarted: true });

    fireEvent.click(screen.getByRole('button', { name: /GÖREV DURDUR/ }));

    expect(value.stopMission).toHaveBeenCalledOnce();
    expect(value.startMission).not.toHaveBeenCalled();
  });

  it('keeps the safe stop exit available even if backend missed mission state', () => {
    const value = renderBar({ missionStarted: false });

    fireEvent.click(screen.getByRole('button', { name: /GÖREV DURDUR/ }));

    expect(value.stopMission).toHaveBeenCalledOnce();
  });
});
