// Overlay data for the LiDAR view, built from published V2X data only (pure; unit tested in Node).
//
// The overlay for a point cloud of frame F_l uses the newest V2X inbox with frame <= F_l: never a newer one, so the
// picture never shows a state the point cloud could not have seen. "Received" stations are drawn where the
// RECEIVED messages say they are, with the age of that information at the point cloud's simulation time.

export function receivedFromInbox(store, stationId, lidarFrame, lidarSimTime, maxAgeS = 2.0) {
  const ring = store.rings.get('v2x.inbox') || [];
  const byTx = new Map();
  let used = null;
  for (const entry of ring) {
    if (entry.frame > lidarFrame) break;
    if (!entry.data || entry.data.station_id !== stationId) continue;
    used = entry;
    for (const m of entry.data.messages) {
      if (m.msg_type === 'BSM') byTx.set(`v${m.tx_id}`, { kind: 'vehicle', m, entry });
      else if (m.msg_type === 'VRUAM') byTx.set(`p${m.tx_id}`, { kind: 'vru', m, entry });
      else if (m.msg_type === 'DENM') byTx.set(`d${m.tx_id}:${m.message.action_id}`, { kind: 'denm', m, entry });
      else if (m.msg_type === 'MAP') byTx.set(`r${m.tx_id}`, { kind: 'rsu', m, entry });
    }
  }
  const received = [], vrus = [], denms = [], rsus = [];
  for (const { kind, m } of byTx.values()) {
    const msg = m.message;
    const age = lidarSimTime - (msg.timestamp ?? msg.reference_time ?? NaN);
    if (Number.isFinite(age) && age > maxAgeS) continue;
    if (kind === 'vehicle') {
      received.push({ id: m.tx_id, x: msg.x_m, y: msg.y_m, z: msg.z_m, heading: msg.heading, length: msg.vehicle_length,
        width: msg.vehicle_width, height: msg.vehicle_height, age, ghost: false });
    } else if (kind === 'rsu') {
      rsus.push({ id: m.tx_id, x: msg.ref_point[0], y: msg.ref_point[1], age });
    } else if (kind === 'vru') {
      vrus.push({ id: m.tx_id, x: msg.position[0], y: msg.position[1], age });
    } else {
      denms.push({ x: msg.event_position[0], y: msg.event_position[1], radius: msg.relevance_distance, age });
    }
  }
  return { received, vrus, denms, rsusReceived: rsus, v2xFrame: used ? used.frame : null, v2xSimTime: used ? used.simTime : null };
}

/** Everything the viewer needs for one point cloud. ``world`` is the latest world.state envelope (observer/admin). */
export function buildOverlays({ store, stationId, lidarFrame, lidarSimTime, world, awareness, includeTruth, ego: egoOverride = null }) {
  // ground truth (world.state) exists only for observer and admin; a vehicle passes its own telemetry as ``ego``
  // and sees RSUs only through the MAP messages it received
  const ego = egoOverride || (world ? world.data.stations.find((s) => s.id === stationId) : null);
  const rx = receivedFromInbox(store, stationId, lidarFrame, lidarSimTime);
  return {
    ego: ego ? { x: ego.x, y: ego.y, z: ego.z, yaw: ego.yaw } : null,
    ...rx,
    truth: includeTruth && world ? world.data.stations.filter((s) => s.id !== stationId).map((s) => ({ id: s.id, kind: s.kind, x: s.x, y: s.y, z: s.z, yaw: s.yaw })) : [],
    rsus: world ? (world.data.rsus || []).map((r) => ({ id: r.station_id, x: r.x, y: r.y })) : rx.rsusReceived,
    awareness: awareness || null,
  };
}
