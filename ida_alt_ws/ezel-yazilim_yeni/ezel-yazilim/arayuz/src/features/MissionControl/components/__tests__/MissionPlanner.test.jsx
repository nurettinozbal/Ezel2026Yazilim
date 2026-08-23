import { describe, expect, it, vi } from 'vitest';
import { fireEvent, render, screen } from '@testing-library/react';
import { VehicleContext } from '@/context/useVehicle';
import MissionPlanner from '@/features/MissionControl/components/MissionPlanner';


const renderPlanner = () => {
  const value = {
    missionPoints: [],
    missionDefaultParkur: 1,
    setMissionDefaultParkur: vi.fn(),
    addMissionPoint: vi.fn(),
    setMissionPointParkur: vi.fn(),
    clearMission: vi.fn(),
    uploadMission: vi.fn(),
    sendCommand: vi.fn(),
    telemetry: {
      system: { logs: [], mission_waypoint_count: 0 },
      ida: { satellites: 0, fix_type: 0 },
    },
  };
  render(
    <VehicleContext.Provider value={value}>
      <MissionPlanner />
    </VehicleContext.Provider>,
  );
  return value;
};


describe('MissionPlanner parkur selection', () => {
  it('changes the default parkur used by map and coordinate additions', () => {
    const value = renderPlanner();

    fireEvent.change(screen.getByRole('combobox', { name: 'Yeni nokta parkuru' }), {
      target: { value: '2' },
    });

    expect(value.setMissionDefaultParkur).toHaveBeenCalledWith(2);
    expect(screen.getByText(/İlk HOME daima P1/)).toBeInTheDocument();
  });
});
