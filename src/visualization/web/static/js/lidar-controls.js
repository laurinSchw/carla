// Request / reconfigure / release form for one LiDAR sensor, generated from GET /api/lidar/schema.
// Used by the vehicle page (own vehicle) and the admin page (any vehicle).

import { fieldsFromLidarSchema, renderForm, readForm, validateAll, valuesToConfig, configToValues } from './schema-form.js';

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

export class LidarConfigForm {
  /**
   * @param {HTMLElement} container
   * @param {object} schema  GET /api/lidar/schema
   * @param {{simRateHz: number}} o
   */
  constructor(container, schema, { simRateHz = 20 } = {}) {
    this.container = container;
    this.schema = schema;
    this.simRateHz = simRateHz;
    this.sensorType = 'ray_cast_semantic';
    this.values = {};
    this.fields = [];
    this.typeSelect = document.createElement('select');
    this.typeSelect.setAttribute('aria-label', 'Sensor type');
    for (const [k, v] of Object.entries(schema.sensor_types)) {
      const o = el('option', '', v.label);
      o.value = k;
      this.typeSelect.appendChild(o);
    }
    this.typeSelect.addEventListener('change', () => { this.sensorType = this.typeSelect.value; this._render(); });
    this.formBox = el('div');
    this.note = el('p', 'stamp');
    this.container.append(el('label', '', 'Sensor type '), this.typeSelect, this.formBox, this.note);
    this._render();
  }

  _render() {
    this.fields = fieldsFromLidarSchema(this.schema, this.sensorType);
    const defaults = configToValues(this.fields, {});
    this.values = { ...defaults, ...this.values };
    renderForm(this.formBox, this.fields, this.values, { onChange: () => this._updateNote(), idPrefix: 'lidar' });
    this._updateNote();
  }

  /** Fill the form from an existing sensor config (reconfigure). */
  setConfig(config) {
    this.sensorType = config.sensor_type || this.sensorType;
    this.typeSelect.value = this.sensorType;
    this.fields = fieldsFromLidarSchema(this.schema, this.sensorType);
    this.values = configToValues(this.fields, config);
    renderForm(this.formBox, this.fields, this.values, { onChange: () => this._updateNote(), idPrefix: 'lidar' });
    this._updateNote();
  }

  /** {config} or {errors}. */
  read() {
    const raw = readForm(this.formBox, this.fields);
    const errors = validateAll(this.fields, raw);
    if (Object.keys(errors).length) return { errors };
    return { config: { sensor_type: this.sensorType, ...valuesToConfig(this.fields, raw) } };
  }

  _updateNote() {
    const r = this.read();
    if (r.errors) { this.note.textContent = `invalid: ${Object.entries(r.errors).map(([k, v]) => `${k} ${v}`).join('; ')}`; return; }
    const c = r.config;
    const pps = c.points_per_second ?? 500000, dt = 1 / this.simRateHz;
    const perFrame = pps * dt;
    const rot = c.rotation_frequency_hz ?? 20;
    this.note.textContent = `${Math.round(perFrame).toLocaleString('en-US')} points per simulation step (${pps.toLocaleString('en-US')} points/s × ${dt} s)`
      + (Math.abs(rot - this.simRateHz) > 1e-6 ? `; rotation ${rot} Hz differs from the step rate ${this.simRateHz} Hz, so a measurement is not one full sweep` : '');
  }
}
