// Forms from JSON schema. The pure part (schema -> field descriptors, values <-> nested config, validation) is
// unit tested in Node; renderForm/readForm are the DOM part.
//
// Two schema flavours are understood:
//  * pydantic JSON schema (GET /api/config/schema): {properties: {name: {type, minimum, maximum, anyOf: [..., null]}}}
//  * the LiDAR schema (GET /api/lidar/schema): {sensor_types: {type: {attributes: [{name, type, min, max, unit}]}}}

const KIND = { integer: 'int', number: 'float', boolean: 'bool', string: 'string' };

function pickType(prop) {
  if (prop.type) return { prop, nullable: false };
  const variants = prop.anyOf || prop.oneOf || [];
  const nonNull = variants.filter((v) => v.type !== 'null');
  return { prop: nonNull[0] || {}, nullable: variants.some((v) => v.type === 'null') };
}

export function fieldsFromJsonSchema(schema) {
  const required = new Set(schema.required || []);
  const out = [];
  for (const [name, raw] of Object.entries(schema.properties || {})) {
    const { prop, nullable } = pickType(raw);
    const kind = prop.enum ? 'enum' : KIND[prop.type] || 'string';
    out.push({
      name,
      label: raw.title || name,
      description: raw.description || '',
      kind,
      nullable,
      required: required.has(name),
      min: prop.minimum ?? raw.minimum ?? null,
      max: prop.maximum ?? raw.maximum ?? null,
      exclusiveMin: prop.exclusiveMinimum ?? raw.exclusiveMinimum ?? null,
      options: prop.enum || null,
      default: raw.default === undefined ? null : raw.default,
      unit: '',
      apply: raw['x-apply'] || null,                 // 'hot' (changeable while running) or 'restart' (needs a new world)
      effect: raw['x-effect'] || '',
    });
  }
  return out;
}

export function fieldsFromLidarSchema(schema, sensorType) {
  const t = schema.sensor_types[sensorType];
  if (!t) throw new Error(`unknown sensor type ${sensorType}`);
  return t.attributes.map((a) => ({
    name: a.name,
    label: a.label,
    description: '',
    kind: a.type === 'int' ? 'int' : a.type === 'bool' ? 'bool' : 'float',
    nullable: a.default === null,
    required: false,
    min: a.min ?? null,
    max: a.max ?? null,
    exclusiveMin: null,
    options: null,
    default: a.default ?? null,
    unit: a.unit || '',
  }));
}

/** Validate one raw input value (string or number). Returns an error text or null. */
export function validateField(field, raw) {
  if (raw === '' || raw === null || raw === undefined) {
    return field.nullable || !field.required && field.default === null ? null : field.kind === 'bool' ? null : 'required';
  }
  if (field.kind === 'bool') return null;
  if (field.kind === 'enum') return field.options.includes(raw) ? null : `one of ${field.options.join(', ')}`;
  if (field.kind === 'string') return null;
  const v = typeof raw === 'number' ? raw : Number(raw);
  if (!Number.isFinite(v)) return 'not a number';
  if (field.kind === 'int' && !Number.isInteger(v)) return 'whole number';
  if (field.min !== null && v < field.min) return `at least ${field.min}`;
  if (field.max !== null && v > field.max) return `at most ${field.max}`;
  if (field.exclusiveMin !== null && v <= field.exclusiveMin) return `greater than ${field.exclusiveMin}`;
  return null;
}

/** Flat {dotted.name: raw} -> nested config object with typed values; empty optional fields are left out. */
export function valuesToConfig(fields, raw) {
  const out = {};
  for (const f of fields) {
    let v = raw[f.name];
    if (v === '' || v === undefined || v === null) {
      if (f.kind === 'bool' && v === false) v = false;
      else continue;
    }
    if (f.kind === 'int' || f.kind === 'float') v = Number(v);
    if (f.kind === 'bool') v = v === true || v === 'true' || v === 'on';
    const parts = f.name.split('.');
    let o = out;
    for (let i = 0; i < parts.length - 1; i++) o = o[parts[i]] = o[parts[i]] || {};
    o[parts[parts.length - 1]] = v;
  }
  return out;
}

/** Nested config -> flat {dotted.name: value} for filling a form. */
export function configToValues(fields, config) {
  const out = {};
  for (const f of fields) {
    let v = config;
    for (const p of f.name.split('.')) v = v == null ? undefined : v[p];
    out[f.name] = v === undefined ? f.default : v;
  }
  return out;
}

export function validateAll(fields, raw) {
  const errors = {};
  for (const f of fields) {
    const e = validateField(f, raw[f.name]);
    if (e) errors[f.name] = e;
  }
  return errors;
}

// ── DOM ─────────────────────────────────────────────────────────────────────────────────────────

/** Render inputs into ``container``. ``badges`` maps a field name to a small tag text such as "hot" / "restart". */
export function renderForm(container, fields, values, { onChange = () => {}, badges = {}, idPrefix = 'f' } = {}) {
  container.textContent = '';
  for (const f of fields) {
    const row = document.createElement('div');
    row.className = 'field';
    const id = `${idPrefix}-${f.name.replace(/\./g, '-')}`;
    const label = document.createElement('label');
    label.htmlFor = id;
    label.textContent = f.label + (f.unit ? ` (${f.unit})` : '');
    if (f.description) label.title = f.description;
    row.appendChild(label);
    let input;
    if (f.kind === 'bool') {
      input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = !!values[f.name];
    } else if (f.kind === 'enum') {
      input = document.createElement('select');
      for (const o of f.options) {
        const opt = document.createElement('option');
        opt.value = o;
        opt.textContent = o;
        input.appendChild(opt);
      }
      input.value = values[f.name] ?? f.default ?? f.options[0];
    } else {
      input = document.createElement('input');
      input.type = f.kind === 'string' ? 'text' : 'number';
      if (f.kind !== 'string') {
        input.step = f.kind === 'int' ? '1' : 'any';
        if (f.min !== null) input.min = String(f.min);
        if (f.max !== null) input.max = String(f.max);
      }
      const v = values[f.name];
      input.value = v === null || v === undefined ? '' : String(v);
      if (f.nullable) input.placeholder = 'default';
    }
    input.id = id;
    input.name = f.name;
    input.addEventListener('input', () => {
      const err = validateField(f, readOne(input, f));
      input.setCustomValidity(err || '');
      input.classList.toggle('invalid', !!err);
      onChange(f.name, readOne(input, f), err);
    });
    row.appendChild(input);
    if (badges[f.name]) {
      const b = document.createElement('span');
      b.className = `badge badge-${badges[f.name]}`;
      b.textContent = badges[f.name];
      row.appendChild(b);
    }
    container.appendChild(row);
  }
}

function readOne(input, f) {
  if (f.kind === 'bool') return input.checked;
  return input.value;
}

export function readForm(container, fields) {
  const raw = {};
  for (const f of fields) {
    const el = container.querySelector(`[name="${CSS.escape(f.name)}"]`);
    if (el) raw[f.name] = readOne(el, f);
  }
  return raw;
}
