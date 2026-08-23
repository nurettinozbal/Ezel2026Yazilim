import { useEffect, useState } from 'react';
import { useVehicle } from '@/context/useVehicle';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer, BarChart, Bar } from 'recharts';
import styles from './styles/EngineeringPage.module.css';

const LAB_ENABLED = ['1', 'true', 'yes', 'on'].includes(String(import.meta.env.VITE_ENABLE_LAB_DEBUG || '').toLowerCase());
const COLORS = { orange: '#f97316', red: '#ef4444', green: '#22c55e', blue: '#3b82f6', black: '#111827' };

const BodyFrameView = ({ snapshot }) => {
  const width = 520; const height = 360; const scale = 22;
  const project = (x, y) => [width / 2 + y * scale, height - 35 - x * scale];
  const lidar = snapshot?.lidar || { status: 'unknown', points: [], clusters: [] };
  const objects = snapshot?.fusion?.objects || [];
  const bearing = snapshot?.camera?.bearing_rad;
  const command = snapshot?.chosen_command || { forward_mps: 0, yaw_rate_rps: 0 };
  const cameraEnd = bearing == null ? null : project(7 * Math.cos(bearing), 7 * Math.sin(bearing));
  const commandEnd = project(Math.max(-1, Math.min(6, command.forward_mps * 5)), command.yaw_rate_rps * 2);
  return <div className={styles.visualCard}>
    <div className={styles.statusLine}><strong>Body frame</strong><span>Lidar: {lidar.status || 'unknown'}</span><span>Kamera bearing: {bearing == null ? 'unknown' : `${bearing.toFixed(2)} rad`}</span></div>
    <svg className={styles.bodySvg} viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Lidar ve fuzyon body frame gorunumu">
      <defs><marker id="debugArrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 z" fill="#38bdf8" /></marker></defs>
      {[2, 4, 6, 8].map((meters) => <circle key={meters} cx={width / 2} cy={height - 35} r={meters * scale} className={styles.rangeCircle} />)}
      <line x1={width / 2} y1={height - 35} x2={width / 2} y2="20" className={styles.axisLine} />
      <line x1="20" y1={height - 35} x2={width - 20} y2={height - 35} className={styles.axisLine} />
      {lidar.points.map((item, index) => { const [cx, cy] = project(item.forward_m, item.lateral_right_m); return <circle key={`${index}-${item.forward_m}-${item.lateral_right_m}`} cx={cx} cy={cy} r="1.8" fill="#94a3b8" />; })}
      {lidar.clusters.map((cluster) => { const [cx, cy] = project(cluster.forward_m, cluster.lateral_right_m); return <circle key={cluster.id} cx={cx} cy={cy} r="6" fill="none" stroke="#a78bfa" />; })}
      {objects.map((object) => { const [cx, cy] = project(object.forward_m, object.lateral_right_m); return <g key={object.id}><circle cx={cx} cy={cy} r="8" fill={COLORS[object.color] || '#facc15'} /><title>{object.id}</title></g>; })}
      {cameraEnd && <line x1={width / 2} y1={height - 35} x2={cameraEnd[0]} y2={cameraEnd[1]} stroke="#fb7185" strokeDasharray="6 4" />}
      <line x1={width / 2} y1={height - 35} x2={commandEnd[0]} y2={commandEnd[1]} stroke="#38bdf8" strokeWidth="3" markerEnd="url(#debugArrow)" />
      <path d={`M ${width / 2 - 9} ${height - 26} L ${width / 2} ${height - 44} L ${width / 2 + 9} ${height - 26} Z`} fill="#e2e8f0" />
      <text x={width / 2 + 8} y="18" className={styles.axisText}>+X ileri</text><text x={width - 58} y={height - 43} className={styles.axisText}>+Y sag</text>
    </svg>
  </div>;
};

const TelemetryCharts = ({ ida }) => {
  const [history, setHistory] = useState([]);
  useEffect(() => {
    const frame = window.requestAnimationFrame(() => setHistory((old) => [...old, {
      time: new Date().toLocaleTimeString('tr-TR', { second: '2-digit', minute: '2-digit' }),
      targetSpeed: ida.target_speed ?? null, realSpeed: ida.speed,
      targetHeading: ida.target_heading ?? null, realHeading: ida.heading,
    }].slice(-20)));
    return () => window.cancelAnimationFrame(frame);
  }, [ida.speed, ida.heading, ida.target_speed, ida.target_heading]);
  const motors = [{ name: 'Sol Thruster', value: ida.motor_left_pct || 0 }, { name: 'Sag Thruster', value: ida.motor_right_pct || 0 }];
  return <div className={styles.chartGrid}>
    <div className={styles.chartCard}><h3 className={styles.chartTitle}>GERCEK HIZ & SETPOINT</h3><div className={styles.lineChartHeight}><ResponsiveContainer><LineChart data={history}><CartesianGrid strokeDasharray="3 3" stroke="#333" /><XAxis dataKey="time" /><YAxis /><Tooltip /><Legend /><Line dataKey="targetSpeed" stroke="#ef4444" name="Setpoint" dot={false} isAnimationActive={false} /><Line dataKey="realSpeed" stroke="#3498db" name="Gercek" dot={false} isAnimationActive={false} /></LineChart></ResponsiveContainer></div></div>
    <div className={styles.chartCard}><h3 className={styles.chartTitle}>HEADING & SETPOINT</h3><div className={styles.lineChartHeight}><ResponsiveContainer><LineChart data={history}><CartesianGrid strokeDasharray="3 3" stroke="#333" /><XAxis dataKey="time" /><YAxis domain={[0, 360]} /><Tooltip /><Legend /><Line dataKey="targetHeading" stroke="#ef4444" name="Setpoint" dot={false} isAnimationActive={false} /><Line dataKey="realHeading" stroke="#2ecc71" name="Gercek" dot={false} isAnimationActive={false} /></LineChart></ResponsiveContainer></div></div>
    <div className={`${styles.chartCard} ${styles.fullWidth}`}><h3 className={styles.chartTitle}>THRUSTER ISTEGI</h3><div className={styles.barChartHeight}><ResponsiveContainer><BarChart layout="vertical" data={motors}><XAxis type="number" domain={[-100, 100]} /><YAxis dataKey="name" type="category" width={100} /><Tooltip /><Bar dataKey="value" fill="#ef4444" isAnimationActive={false} /></BarChart></ResponsiveContainer></div></div>
  </div>;
};

const FixtureTestRow = ({ test, runPassiveTest }) => {
  const [fixture, setFixture] = useState({ case: '', color: '', range: '', bearing: '', rangeTolerance: '0.25', bearingTolerance: '3', expectedId: '' });
  const required = test.requires_expectations;
  const numeric = (value, minimum, maximum) => value !== '' && Number.isFinite(Number(value)) && Number(value) >= minimum && Number(value) <= maximum;
  const tolerancesValid = numeric(fixture.rangeTolerance, 0.01, 5) && numeric(fixture.bearingTolerance, 0.1, 30);
  const optionalNumbersValid = (!fixture.range || numeric(fixture.range, 0.05, 100)) && (!fixture.bearing || numeric(fixture.bearing, -180, 180));
  const valid = !required || (fixture.case && tolerancesValid && optionalNumbersValid && (fixture.case !== 'positive' || (
    numeric(fixture.range, 0.05, 100) && numeric(fixture.bearing, -180, 180) && (!test.allowed_colors?.length || fixture.color)
  )));
  const start = () => runPassiveTest(test, required ? {
    profile: test.default_profile, case: fixture.case,
    expected_color: fixture.color || null,
    expected_range_m: fixture.range === '' ? null : Number(fixture.range),
    expected_bearing_deg: fixture.bearing === '' ? null : Number(fixture.bearing),
    range_tolerance_m: Number(fixture.rangeTolerance),
    bearing_tolerance_deg: Number(fixture.bearingTolerance),
    expected_id: fixture.expectedId || null,
  } : null);
  const field = (key, value) => setFixture((old) => ({ ...old, [key]: value }));
  return <div className={styles.testRow}><div><strong>{test.id} · {test.label}</strong><small>{test.description}</small>{required && <div>
    <label>Profil <input value={test.default_profile} readOnly /></label>
    <label>Case <select value={fixture.case} onChange={(event) => field('case', event.target.value)}><option value="">Sec</option>{test.allowed_cases.map((item) => <option key={item}>{item}</option>)}</select></label>
    {test.allowed_colors?.length > 0 && <label>Renk <select value={fixture.color} onChange={(event) => field('color', event.target.value)}><option value="">Sec</option>{test.allowed_colors.map((item) => <option key={item}>{item}</option>)}</select></label>}
    <label>Menzil m <input type="number" value={fixture.range} onChange={(event) => field('range', event.target.value)} /></label>
    <label>Bearing deg <input type="number" value={fixture.bearing} onChange={(event) => field('bearing', event.target.value)} /></label>
    <label>Menzil toleransi <input type="number" value={fixture.rangeTolerance} onChange={(event) => field('rangeTolerance', event.target.value)} /></label>
    <label>Bearing toleransi <input type="number" value={fixture.bearingTolerance} onChange={(event) => field('bearingTolerance', event.target.value)} /></label>
    <label>Beklenen ID <input value={fixture.expectedId} onChange={(event) => field('expectedId', event.target.value)} /></label>
  </div>}</div><button type="button" disabled={!valid} onClick={start}>Baslat</button></div>;
};

const LabEngineering = () => {
  const { telemetry, debugSnapshot, vehicleTests, vehicleTestResult, vehicleTestEvents, runPassiveTest, cancelPassiveTest } = useVehicle();
  const [tab, setTab] = useState('telemetry');
  const [clock, setClock] = useState(() => window.performance.now() / 1000);
  useEffect(() => {
    const timer = window.setInterval(() => setClock(window.performance.now() / 1000), 1000);
    return () => window.clearInterval(timer);
  }, []);
  const elapsed = debugSnapshot ? clock - debugSnapshot.client_received_monotonic : Infinity;
  const receiveAge = (debugSnapshot?.server_receive_age_s ?? Infinity) + elapsed;
  const sourceAge = (debugSnapshot?.source_age_s ?? Infinity) + elapsed;
  const stale = !debugSnapshot || !Number.isFinite(elapsed) || elapsed < 0 || debugSnapshot.clock_valid === false || receiveAge < 0 || sourceAge < 0 || receiveAge > 2 || sourceAge > 2;
  const debugAutonomyFresh = !stale && debugSnapshot?.autonomy?.status === 'ok';
  const autonomyFresh = debugAutonomyFresh || telemetry.ida.autonomy_fresh;
  const autonomy = debugAutonomyFresh ? debugSnapshot.autonomy : {
    state: telemetry.ida.autonomy_state, action: telemetry.ida.autonomy_action,
    current_waypoint: telemetry.ida.current_wp, failsafe_reason: '',
  };
  const activeRuns = [...vehicleTestEvents].reverse().filter((event, index, all) => (
    event.run_id && all.findIndex((item) => item.run_id === event.run_id) === index
    && ['pending', 'forwarded', 'cancel_requested'].includes(event.event)
  ));
  return <div className={styles.pageContainer}>
    <div className={styles.tabs}><button type="button" className={tab === 'telemetry' ? styles.activeTab : ''} onClick={() => setTab('telemetry')}>Telemetri</button><button type="button" className={tab === 'perception' ? styles.activeTab : ''} onClick={() => setTab('perception')}>Algi / Fuzyon</button><button type="button" className={tab === 'tests' ? styles.activeTab : ''} onClick={() => setTab('tests')}>Pasif Testler</button></div>
    {tab === 'telemetry' && <TelemetryCharts ida={telemetry.ida} />}
    {tab === 'perception' && <><div className={stale ? styles.staleBanner : styles.freshBanner}>LAB-ONLY · Ham goruntu yok · {stale ? 'STALE' : 'FRESH'} · receive {Number.isFinite(receiveAge) ? receiveAge.toFixed(1) : '∞'}s · source {Number.isFinite(sourceAge) ? sourceAge.toFixed(1) : '∞'}s</div><BodyFrameView snapshot={debugSnapshot} /><div className={styles.autonomyCard}><span>{debugAutonomyFresh ? 'DEBUG OTONOMI FRESH' : telemetry.ida.autonomy_fresh ? 'MAVLINK OTONOMI FRESH' : 'OTONOMI STALE'}</span><span>State: {autonomyFresh ? (autonomy.state || 'unknown') : 'unknown (stale)'}</span><span>Action: {autonomyFresh ? (autonomy.action || 'unknown') : 'unknown (stale)'}</span><span>WP: {autonomyFresh ? (autonomy.current_waypoint ?? 'unknown') : 'unknown'}</span><span>Failsafe: {autonomyFresh ? (autonomy.failsafe_reason || 'yok/unknown') : 'unknown (stale)'}</span></div></>}
    {tab === 'tests' && <div className={styles.testPanel}><div className={styles.debugBanner}>Manifest ida_vehicle_test v1 · {vehicleTests.length} test · sonuc yalniz Jetson monitorunden gelir.</div>{vehicleTests.length === 0 && <p>Backend manifest bekleniyor.</p>}{vehicleTests.map((test) => <FixtureTestRow key={test.id} test={test} runPassiveTest={runPassiveTest} />)}{activeRuns.map((run) => <div className={styles.runRow} key={run.run_id}><code>{run.run_id}</code><span>seq={run.seq} · {run.event}</span><button type="button" disabled={run.event === 'cancel_requested'} onClick={() => cancelPassiveTest(run.run_id)}>Iptal</button></div>)}{vehicleTestEvents.length > 0 && <pre className={styles.testResult}>{JSON.stringify(vehicleTestEvents, null, 2)}</pre>}{vehicleTestResult && <div className={styles.resultSummary}>Son olay: {vehicleTestResult.event} · run={vehicleTestResult.run_id || 'yok'} · seq={vehicleTestResult.seq ?? 'yok'} · status={vehicleTestResult.status}</div>}</div>}
  </div>;
};

const EngineeringPage = () => {
  const { telemetry } = useVehicle();
  if (!LAB_ENABLED) return <div className={styles.pageContainer}><TelemetryCharts ida={telemetry.ida} /></div>;
  return <LabEngineering />;
};

export default EngineeringPage;
