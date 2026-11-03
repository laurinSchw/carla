import test from 'node:test';
import assert from 'node:assert/strict';
import { Store } from '../../src/web/static/js/store.js';
import { receivedFromInbox, buildOverlays } from '../../src/web/static/js/lidar-overlays.js';

const bsm = (id, x, y, ts) => ({ tx_id: id, msg_type: 'BSM', message: { x_m: x, y_m: y, z_m: 0, heading: 90, timestamp: ts, vehicle_length: 4.5, vehicle_width: 1.8, vehicle_height: 1.5 }, link: {} });
const inbox = (frame, simTime, messages, stationId = 7) => ({ topic: 'v2x.inbox', seq: frame, frame, sim_time: simTime, data: { station_id: stationId, messages } });

test('overlay uses the newest inbox NOT NEWER than the point cloud, with the age at the cloud\'s time', () => {
  const s = new Store({ ringSimSeconds: 10 });
  s.put(inbox(100, 5.0, [bsm(1, 10, 0, 5.0)]));
  s.put(inbox(110, 5.5, [bsm(1, 15, 0, 5.5)]));
  s.put(inbox(120, 6.0, [bsm(1, 20, 0, 6.0)]));
  const o = receivedFromInbox(s, 7, 113, 5.65);
  assert.equal(o.v2xFrame, 110);
  assert.equal(o.received.length, 1);
  assert.equal(o.received[0].x, 15, 'the position of the message of frame 110, not of 120');
  assert.ok(Math.abs(o.received[0].age - 0.15) < 1e-9);
});

test('a station not heard in the latest update stays at its last received position, aging; old ones vanish', () => {
  const s = new Store({ ringSimSeconds: 10 });
  s.put(inbox(100, 5.0, [bsm(1, 10, 0, 5.0), bsm(2, 50, 0, 5.0)]));
  s.put(inbox(110, 5.5, [bsm(1, 12, 0, 5.5)]));
  let o = receivedFromInbox(s, 7, 110, 5.5);
  const two = o.received.find((e) => e.id === 2);
  assert.equal(two.x, 50); assert.ok(Math.abs(two.age - 0.5) < 1e-9);
  o = receivedFromInbox(s, 7, 110, 8.0);                         // 3 s later: both older than 2 s
  assert.equal(o.received.length, 0);
});

test('nothing at or before the frame: no V2X frame, empty overlay (not a made-up one)', () => {
  const s = new Store();
  s.put(inbox(200, 10, [bsm(1, 0, 0, 10)]));
  const o = receivedFromInbox(s, 7, 100, 5);
  assert.equal(o.v2xFrame, null); assert.deepEqual(o.received, []);
});

test('only the inbox of the sensor vehicle counts', () => {
  const s = new Store();
  s.put(inbox(100, 5, [bsm(1, 0, 0, 5)], 99));
  assert.equal(receivedFromInbox(s, 7, 100, 5).v2xFrame, null);
});

test('VRUs and DENMs come from received messages', () => {
  const s = new Store();
  s.put(inbox(100, 5, [
    { tx_id: 300, msg_type: 'VRUAM', message: { position: [8, 9, 0], timestamp: 5 }, link: {} },
    { tx_id: -1000001, msg_type: 'DENM', message: { action_id: 1, event_position: [50, 60, 0], relevance_distance: 120, reference_time: 4.9 }, link: {} },
  ]));
  const o = receivedFromInbox(s, 7, 100, 5);
  assert.deepEqual(o.vrus.map((v) => [v.x, v.y]), [[8, 9]]);
  assert.deepEqual(o.denms.map((d) => [d.x, d.y, d.radius]), [[50, 60, 120]]);
});

test('buildOverlays: ground truth only when allowed, ego from world.state, RSUs from world.state', () => {
  const s = new Store();
  s.put(inbox(100, 5, [bsm(1, 0, 0, 5)]));
  const world = { data: { stations: [{ id: 7, kind: 'vehicle', x: 1, y: 2, z: 0, yaw: 30 }, { id: 8, kind: 'vehicle', x: 9, y: 9, z: 0, yaw: 0 }], rsus: [{ station_id: -1, x: 5, y: 5 }] } };
  const a = buildOverlays({ store: s, stationId: 7, lidarFrame: 100, lidarSimTime: 5, world, awareness: 75, includeTruth: true });
  assert.deepEqual(a.ego, { x: 1, y: 2, z: 0, yaw: 30 }); assert.deepEqual(a.truth.map((t) => t.id), [8]); assert.equal(a.rsus[0].id, -1); assert.equal(a.awareness, 75);
  const b = buildOverlays({ store: s, stationId: 7, lidarFrame: 100, lidarSimTime: 5, world, includeTruth: false });
  assert.deepEqual(b.truth, []);
  const c = buildOverlays({ store: s, stationId: 7, lidarFrame: 100, lidarSimTime: 5, world: null, includeTruth: false });
  assert.equal(c.ego, null);
});

test('a vehicle has no world.state: ego from its own telemetry, RSUs only from received MAP messages, no ground truth', () => {
  const s = new Store();
  s.put(inbox(100, 5, [
    bsm(1, 0, 0, 5),
    { tx_id: -3, msg_type: 'MAP', message: { intersection_id: 3, ref_point: [5, 6, 0], timestamp: 4.5 }, link: {} },
  ]));
  const o = buildOverlays({ store: s, stationId: 7, lidarFrame: 100, lidarSimTime: 5, world: null, includeTruth: true,
    ego: { x: 3, y: 4, z: 0, yaw: 90 } });
  assert.deepEqual(o.ego, { x: 3, y: 4, z: 0, yaw: 90 });
  assert.deepEqual(o.truth, [], 'without world.state there is no ground truth, whatever the caller asks for');
  assert.deepEqual(o.rsus.map((r) => [r.id, r.x, r.y]), [[-3, 5, 6]]);
});
